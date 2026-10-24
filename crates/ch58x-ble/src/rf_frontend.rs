//! CH58x RF analog front-end initialization and per-die calibration.
//!
//! This is a clean register-level implementation. The measurements are read
//! from the local die at runtime and converted by the portable interpolation
//! routine in `ble-controller`; no vendor BLE stack is linked or called.

use ble_controller::{Error, rf_calibration};

use crate::radio::{RFEND_BASE, Register};

const TUNE_COUNTDOWN: u32 = 0x1f40;

const FILTER_COUNTDOWN: u32 = 0x64;
const MAX_CALIBRATION_POLLS: usize = 32_768;

#[derive(Clone, Copy)]
pub(crate) struct FrontEnd {
    control_04: Register<u32>,
    analog_enable_08: Register<u32>,
    reset_control_0c: Register<u32>,
    config_28: Register<u32>,
    config_2c: Register<u32>,
    config_30: Register<u32>,
    config_38: Register<u32>,
    config_3c: Register<u32>,
    config_48: Register<u32>,
    config_4c: Register<u32>,
    filter_50: Register<u32>,

    adc_58: Register<u32>,

    config_5c: Register<u32>,
    tune_co_90: Register<u32>,
    tune_ga_94: Register<u32>,
    filter_status_9c: Register<u32>,
    tune_table: [Register<u32>; rf_calibration::TABLE_WORDS],
    lle_operation_countdown: Register<u32>,
}

impl FrontEnd {
    pub(crate) const unsafe fn new(lle_operation_countdown: Register<u32>) -> Self {
        Self {
            control_04: unsafe { Register::at(RFEND_BASE, 0x04) },
            analog_enable_08: unsafe { Register::at(RFEND_BASE, 0x08) },
            reset_control_0c: unsafe { Register::at(RFEND_BASE, 0x0c) },
            config_28: unsafe { Register::at(RFEND_BASE, 0x28) },
            config_2c: unsafe { Register::at(RFEND_BASE, 0x2c) },
            config_30: unsafe { Register::at(RFEND_BASE, 0x30) },
            config_38: unsafe { Register::at(RFEND_BASE, 0x38) },
            config_3c: unsafe { Register::at(RFEND_BASE, 0x3c) },
            config_48: unsafe { Register::at(RFEND_BASE, 0x48) },
            config_4c: unsafe { Register::at(RFEND_BASE, 0x4c) },
            filter_50: unsafe { Register::at(RFEND_BASE, 0x50) },

            adc_58: unsafe { Register::at(RFEND_BASE, 0x58) },

            config_5c: unsafe { Register::at(RFEND_BASE, 0x5c) },
            tune_co_90: unsafe { Register::at(RFEND_BASE, 0x90) },
            tune_ga_94: unsafe { Register::at(RFEND_BASE, 0x94) },
            filter_status_9c: unsafe { Register::at(RFEND_BASE, 0x9c) },
            tune_table: [
                unsafe { Register::at(RFEND_BASE, 0xa0) },
                unsafe { Register::at(RFEND_BASE, 0xa4) },
                unsafe { Register::at(RFEND_BASE, 0xa8) },
                unsafe { Register::at(RFEND_BASE, 0xac) },
                unsafe { Register::at(RFEND_BASE, 0xb0) },
                unsafe { Register::at(RFEND_BASE, 0xb4) },
                unsafe { Register::at(RFEND_BASE, 0xb8) },
                unsafe { Register::at(RFEND_BASE, 0xbc) },
                unsafe { Register::at(RFEND_BASE, 0xc0) },
                unsafe { Register::at(RFEND_BASE, 0xc4) },
                unsafe { Register::at(RFEND_BASE, 0xc8) },
                unsafe { Register::at(RFEND_BASE, 0xcc) },
                unsafe { Register::at(RFEND_BASE, 0xd0) },
            ],
            lle_operation_countdown,
        }
    }

    pub(crate) fn initialize_baseline(&self) {
        {
            // Register-for-register reconstruction of RFEND_DevInit from the
            // CH585 public RF_basic archive. Unlike CH582 it does not perform
            // the 0x1101/0/0x1101 analog reset sequence here.
            self.config_28.write(0x480);
            self.config_3c
                .modify(|value| (value & !0x0600_0000) | 0x0200_0000);
            self.config_3c
                .modify(|value| (value & !0xe000_0000) | 0x4000_0000);
            self.config_30.modify(|value| value | 0x0000_0700);
            self.config_30.modify(|value| value | 0x0000_7000);
            self.config_30.modify(|value| value | 0x0007_0000);
            self.config_30
                .modify(|value| (value & !0x7000_0000) | 0x1000_0000);
            self.config_48.modify(|value| value & !0x0007_0000);
            self.config_4c.modify(|value| value & !0x0000_0007);
            self.config_4c.modify(|value| value & !0x0000_0070);
            self.config_4c.modify(|value| value & !0x0003_0000);
            self.config_5c.modify(|value| value & !0x0007_0000);
            self.config_5c.modify(|value| value & !0x0070_0000);
            for _ in 0..100 {
                core::hint::spin_loop();
            }
        }
        self.snapshot(1, 0);
    }

    pub(crate) fn calibrate(&self) -> Result<(), Error> {
        self.analog_enable_08.modify(|value| value | 0x0033_0000);

        if let Err(error) = self.tx_ctune() {
            self.snapshot(
                0x8000_0003,
                match error {
                    Error::Timeout => 1,
                    Error::Hardware => 2,
                    _ => 3,
                },
            );
            return Err(error);
        }

        self.tx_ftune();

        self.rx_filter();

        self.rx_adc();
        self.snapshot(8, 0);
        Ok(())
    }

    pub(crate) fn leave_calibration(&self) {
        // Keep the RX ADC calibration enable bit set, matching CH582 idle.
        self.analog_enable_08.modify(|value| value & 0xffcd_ffff);
    }

    pub(crate) fn record_terminal(&self, error: Option<Error>) {
        let error_code = match error {
            None => 0,
            Some(Error::Timeout) => 1,
            Some(Error::Hardware) => 2,
            Some(_) => 3,
        };
        self.snapshot(if error_code == 0 { 9 } else { 0x8000_0009 }, error_code);
    }

    fn tx_ctune(&self) -> Result<(), Error> {
        self.control_04.modify(|value| value & !0x100);
        self.config_28.modify(|value| value & !0x1000);
        self.config_2c.modify(|value| value & !0x10);
        self.analog_enable_08.modify(|value| value | 0x0002_0000);
        self.control_04.modify(|value| value | 0x10);

        let measurements = (|| {
            let mhz_2401 = self.measure_ctune(0x0000_bf00)?;

            let mhz_2480 = self.measure_ctune(0x0000_e800)?;

            let mhz_2440 = self.measure_ctune(0x0000_d400)?;

            let _mhz_2522 = self.measure_ctune(0x0000_fc00)?;

            Ok((mhz_2401, mhz_2440, mhz_2480))
        })();

        self.control_04.modify(|value| value & !0x11);
        self.config_28.modify(|value| value | 0x1000);
        self.config_2c.modify(|value| value | 0x10);

        let (mhz_2401, mhz_2440, mhz_2480) = measurements?;
        self.config_38.modify(|value| {
            (value & 0x80ff_ffc0)
                | u32::from(mhz_2440.co & 0x3f)
                | (u32::from(mhz_2440.ga & 0x7f) << 24)
        });

        let table = rf_calibration::build_table(mhz_2401, mhz_2440, mhz_2480)
            .map_err(|_| Error::Hardware)?;

        for (index, (register, value)) in self.tune_table.iter().zip(table).enumerate() {
            if index + 1 == rf_calibration::TABLE_WORDS {
                register.modify(|old| (old & 0xf000_0000) | (value & 0x0fff_ffff));
            } else {
                register.write(value);
            }
        }
        Ok(())
    }

    fn measure_ctune(&self, tune: u32) -> Result<rf_calibration::Measurement, Error> {
        self.control_04.modify(|value| value & !0x01);
        self.config_38.modify(|value| (value & 0xfffe_00ff) | tune);
        self.control_04.modify(|value| value | 0x01);
        self.wait_tune()?;
        Ok(rf_calibration::Measurement {
            co: (self.tune_co_90.read() & 0x3f) as u8,
            ga: ((self.tune_ga_94.read() >> 10) & 0x7f) as u8,
        })
    }

    fn tx_ftune(&self) {
        // C-tune leaves RFEND_CONTROL[8] clear. Frequency tuning is latched by
        // setting it once after the compensation table has been populated.
        self.control_04.modify(|value| value | 0x100);
    }

    fn wait_tune(&self) -> Result<(), Error> {
        self.lle_operation_countdown.write(TUNE_COUNTDOWN);
        for _ in 1..=MAX_CALIBRATION_POLLS {
            if self.tune_co_90.read() & 0x0600_0000 == 0x0600_0000 {
                return Ok(());
            }
            if self.lle_operation_countdown.read() == 0 {
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }

    fn rx_filter(&self) {
        // RFEND_RXFilter clears the complete four-bit calibration field at
        // bits 16..19 before starting the analog measurement.

        // CH585 clears only the calibration-enable bit before measuring; the
        // other three bits in this field retain their analog trim state.

        self.filter_50.modify(|value| value & !0x0001_0000);
        self.analog_enable_08.modify(|value| value | 0x0020_0000);
        self.reset_control_0c.modify(|value| value | 0x10);
        self.reset_control_0c.modify(|value| value & !0x10);
        for _ in 0..4 {
            core::hint::spin_loop();
        }
        self.reset_control_0c.modify(|value| value | 0x10);
        self.control_04.modify(|value| value | 0x1000);
        self.lle_operation_countdown.write(FILTER_COUNTDOWN);

        for _ in 1..=MAX_CALIBRATION_POLLS {
            if self.filter_status_9c.read() & 0x100 != 0 {
                break;
            }
            if self.lle_operation_countdown.read() == 0 {
                break;
            }
            core::hint::spin_loop();
        }

        for _ in 0..4 {
            core::hint::spin_loop();
        }
        let result = self.filter_status_9c.read() & 0x1f;
        self.filter_50.modify(|value| value | 0x0001_0000);
        self.filter_50.modify(|value| (value & !0x1f) | result);
        self.analog_enable_08.modify(|value| value & !0x0020_0000);
    }

    fn rx_adc(&self) {
        self.adc_58.modify(|value| value & !0x0001_0000);
        self.analog_enable_08.modify(|value| value | 0x0001_0000);
        self.reset_control_0c.modify(|value| value & !0x100);
        self.control_04.modify(|value| value & !0x0001_0000);
        self.reset_control_0c.modify(|value| value | 0x100);
        self.control_04.modify(|value| value | 0x0001_0000);
    }

    #[inline(always)]
    fn snapshot(&self, _stage: u32, _error: u32) {}
}

//! Register-level CH58x ADC support.
//!
//! The configuration model and channel numbering follow the reusable,
//! non-vendor-stack portions of `ch32-rs/ch58x-hal` at revision `611954e`.
//! Conversion waits are bounded so a failed ADC cannot block firmware
//! indefinitely.

use crate::pac;

/// ADC sampling clock derived from the 32 MHz peripheral clock.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum SamplingClock {
    #[default]
    Mhz3_2 = 0b00,
    Mhz2_67 = 0b01,
    Mhz5_33 = 0b10,
    Mhz4 = 0b11,
}

/// Programmable ADC input gain.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum Gain {
    Minus12Db = 0b00,
    #[default]
    Minus6Db = 0b01,
    ZeroDb = 0b10,
    Plus6Db = 0b11,
}

/// ADC input channel index.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Channel {
    Pa4 = 0,
    Pa5 = 1,
    Pa12 = 2,
    Pa13 = 3,
    Pa14 = 4,
    Pa15 = 5,
    Pa3 = 6,
    Pa2 = 7,
    Pa1 = 8,
    Pa0 = 9,
    Pa6 = 10,
    Pa7 = 11,
    Pa8 = 12,
    Pa9 = 13,
    Internal = 15,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub clock: SamplingClock,
    pub gain: Gain,
    pub differential: bool,
    pub buffer_enabled: bool,
    pub offset_test: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock: SamplingClock::Mhz3_2,
            gain: Gain::Minus6Db,
            differential: false,
            buffer_enabled: true,
            offset_test: false,
        }
    }
}

impl Config {
    pub const fn temperature() -> Self {
        Self {
            clock: SamplingClock::Mhz3_2,
            gain: Gain::Plus6Db,
            differential: true,
            buffer_enabled: false,
            offset_test: false,
        }
    }

    pub const fn offset_noise() -> Self {
        Self {
            clock: SamplingClock::Mhz3_2,
            gain: Gain::Minus6Db,
            differential: false,
            buffer_enabled: false,
            offset_test: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Timeout,
}

pub struct Adc {
    _adc: pac::ADC,
}

impl Adc {
    pub fn new(adc: pac::ADC, config: Config) -> Self {
        let this = Self { _adc: adc };
        this.set_config(config);
        this
    }

    pub fn set_config(&self, config: Config) {
        let registers = unsafe { &*pac::ADC::PTR };
        registers.cfg().modify(|_, w| unsafe {
            w.power_on()
                .set_bit()
                .diff_en()
                .bit(config.differential)
                .clk_div()
                .bits(config.clock as u8)
                .buf_en()
                .bit(config.buffer_enabled)
                .pga_gain()
                .bits(config.gain as u8)
                .ofs_test()
                .bit(config.offset_test)
        });
    }

    pub fn enable_temperature(&self) {
        let registers = unsafe { &*pac::ADC::PTR };
        registers.tem_sensor().modify(|_, w| w.power_on().set_bit());
    }

    pub fn disable_temperature(&self) {
        let registers = unsafe { &*pac::ADC::PTR };
        registers
            .tem_sensor()
            .modify(|_, w| w.power_on().clear_bit());
    }

    /// Performs one bounded blocking conversion.
    pub fn read(&mut self, channel: Channel) -> Result<u16, Error> {
        const MAX_POLLS: usize = 4096;
        let registers = unsafe { &*pac::ADC::PTR };
        registers
            .channel()
            .write(|w| unsafe { w.ch_idx().bits(channel as u8) });
        registers.convert().modify(|_, w| w.start().set_bit());
        for _ in 0..MAX_POLLS {
            if registers.convert().read().start().bit_is_clear() {
                return Ok(registers.data().read().data().bits());
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }
}

impl Drop for Adc {
    fn drop(&mut self) {
        let registers = unsafe { &*pac::ADC::PTR };
        registers
            .tem_sensor()
            .modify(|_, w| w.power_on().clear_bit());
        registers.cfg().modify(|_, w| w.power_on().clear_bit());
    }
}

//! Register-level CH58x 2.4 GHz radio and BLE baseband driver.
//!
//! The public PAC omits this block. Addresses and bit meanings are admitted
//! only after independent confirmation from multiple observations or a focused
//! hardware experiment.

use crate::radio_phy::*;
use core::ptr::{read_volatile, write_volatile};

use ble_controller::{BleRadio, Channel, Error, Phy, ReceivedPacket};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use crate::radio_dma_policy::{
    CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN, ReceiveLengthPolicy, classify_received_length,
};
pub use crate::radio_dma_policy::{
    MAX_LEGACY_ADV_PAYLOAD_LEN, MAX_LL_DATA_PAYLOAD_LEN, RF_DMA_BUFFER_LEN,
    SUPPORTS_DATA_LENGTH_EXTENSION, SUPPORTS_EXTENDED_ADVERTISING,
};

const BB_BASE: usize = 0x4000_c100;

const LLE_BASE: usize = 0x4000_c200;

pub(crate) const RFEND_BASE: usize = 0x4000_d000;

const TX_POWER_0_DBM: u8 = 0x15;
const BB_TX_POWER_MASK: u32 = 0x7e00_0000;
// CH585 BB status bits 4..=8 are write-one-to-clear.
const BB_INTERRUPT_STATUS_MASK: u32 = 0x0000_01f0;

#[path = "radio_rx_status.rs"]
mod rx_status;
const MAX_OPERATION_POLLS: usize = 65_536;

// CH585 uses the same low RX/TX state encodings as the older CH58x LLE, but
// places them under a new 0x30000 mode bank. These complete values and the
// BB[9:8] mode field are taken from the public CH585 RF_basic archive.

const BB_MODE_MASK: u32 = 0x300;

const BB_MODE_IDLE: u32 = 0x100;

const BB_MODE_ACTIVE: u32 = 0x200;

const LLE_MODE_IDLE: u32 = 0x0003_0000;

const LLE_MODE_CALIBRATION: u32 = 0x0003_0558;

const LLE_MODE_RX: u32 = 0x0003_0158;

const LLE_MODE_TX: u32 = 0x0003_0258;
const LLE_TX_BUFFER_OFFSET: usize = 0x78;

const LLE_RX_BUFFER_OFFSET: usize = 0x7c;

const LLE_STATUS_RX_COMPLETE: u32 = 1 << 0;
const LLE_STATUS_TX_COMPLETE: u32 = 1 << 1;
const LLE_STATUS_RX_TO_TX: u32 = 1 << 2;
const LLE_STATUS_TX_TO_RX: u32 = 1 << 3;

const LLE_STATUS_RX_CAPTURE_COMPLETE: u32 = 1 << 17;
const LLE_STATUS_TIMING: u32 = LLE_STATUS_RX_CAPTURE_COMPLETE;

const LLE_1M_TRANSITION_COUNTDOWN: u32 = 0x18e;
// Legacy advertising accepts at most a 37-byte payload. This is the capture
// countdown bound, not the allocation size.
const LEGACY_ADV_RX_MAX_LEN: usize = MAX_LEGACY_ADV_PAYLOAD_LEN;
pub(crate) const CONNECTION_EVENT_RX_BUFFER_LEN: usize = 2 + CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN;
const _: () = assert!(CONNECTION_EVENT_RX_BUFFER_LEN <= RF_DMA_BUFFER_LEN);

// A CONNECT_IND starts one 150-us IFS after ADV_IND and occupies 352 us at
// LE 1M (preamble + AA + 2-byte header + 34-byte payload + CRC).  A 500-us
// receive window therefore cuts the packet off just before its CRC finishes.
// Keep measured setup/clock margin beyond the spec-derived 502-us minimum.
const LEGACY_ADV_RESPONSE_WINDOW_US: u64 = 650;

#[repr(C, align(4))]
struct DmaBuffer([u8; RF_DMA_BUFFER_LEN]);

static mut TX_BUFFER: DmaBuffer = DmaBuffer([0; RF_DMA_BUFFER_LEN]);
static mut RESPONSE_BUFFER: DmaBuffer = DmaBuffer([0; RF_DMA_BUFFER_LEN]);
static mut RX_BUFFER: DmaBuffer = DmaBuffer([0; RF_DMA_BUFFER_LEN]);

const RF_DMA_LIMITS_MAGIC: u32 = 0x5246_4c31; // "RFL1"
/// Bounded telemetry for the deliberately reduced RF DMA capability.
/// Words are magic, a claimed wire length exceeding the physical DMA bank,
/// and a packet exceeding the enabled legacy/pre-DLE protocol capability.
#[used]
#[unsafe(export_name = "CH58X_BLE_RF_DMA_LIMIT_DIAGNOSTICS")]
#[unsafe(link_section = ".uninit.ch58x_ble_rf_dma_limits")]
static RF_DMA_LIMIT_DIAGNOSTICS: [AtomicU32; 3] = [const { AtomicU32::new(0) }; 3];

/// Return `(physical_dma_overflow, unsupported_packet_length)` counters.
pub fn rf_dma_limit_counters() -> (u32, u32) {
    if RF_DMA_LIMIT_DIAGNOSTICS[0].load(Ordering::Acquire) != RF_DMA_LIMITS_MAGIC {
        return (0, 0);
    }
    (
        RF_DMA_LIMIT_DIAGNOSTICS[1].load(Ordering::Relaxed),
        RF_DMA_LIMIT_DIAGNOSTICS[2].load(Ordering::Relaxed),
    )
}

static CAPTURE_SEEN: AtomicBool = AtomicBool::new(false);
static RX_CAPTURE_COUNTDOWN: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
pub(crate) struct Register<T> {
    address: *mut T,
}

unsafe impl<T> Send for Register<T> {}

impl Register<u32> {
    pub(crate) const unsafe fn at(base: usize, offset: usize) -> Self {
        Self {
            address: (base + offset) as *mut u32,
        }
    }

    pub(crate) fn read(self) -> u32 {
        unsafe { read_volatile(self.address) }
    }

    pub(crate) fn write(self, value: u32) {
        unsafe { write_volatile(self.address, value) }
    }

    pub(crate) fn modify(self, f: impl FnOnce(u32) -> u32) {
        self.write(f(self.read()));
    }
}

#[derive(Clone, Copy)]
struct RadioRegisters {
    bb_config: Register<u32>,
    bb_tx_crc_init: Register<u32>,
    bb_tx_access_address: Register<u32>,
    bb_receive_config_10: Register<u32>,

    bb_receive_config_18: Register<u32>,

    bb_tx_control: Register<u32>,
    bb_rssi_status: Register<u32>,
    bb_interrupt_enable: Register<u32>,
    bb_interrupt_status: Register<u32>,

    bb_config_3c: Register<u32>,
    lle_command: Register<u32>,
    lle_config: Register<u32>,
    lle_status: Register<u32>,
    lle_interrupt_enable: Register<u32>,
    lle_rx_to_tx_timing: Register<u32>,
    lle_mode: Register<u32>,
    lle_operation_countdown: Register<u32>,
    lle_tx_buffer: Register<u32>,
    lle_rx_buffer: Register<u32>,
}

impl RadioRegisters {
    const unsafe fn new() -> Self {
        Self {
            bb_config: unsafe { Register::at(BB_BASE, 0x00) },
            bb_tx_crc_init: unsafe { Register::at(BB_BASE, 0x04) },
            bb_tx_access_address: unsafe { Register::at(BB_BASE, 0x08) },
            bb_receive_config_10: unsafe { Register::at(BB_BASE, 0x10) },

            bb_receive_config_18: unsafe { Register::at(BB_BASE, 0x18) },

            bb_tx_control: unsafe { Register::at(BB_BASE, 0x2c) },
            bb_rssi_status: unsafe { Register::at(BB_BASE, 0x30) },
            bb_interrupt_enable: unsafe { Register::at(BB_BASE, 0x34) },
            bb_interrupt_status: unsafe { Register::at(BB_BASE, 0x38) },

            bb_config_3c: unsafe { Register::at(BB_BASE, 0x3c) },
            lle_command: unsafe { Register::at(LLE_BASE, 0x00) },
            lle_config: unsafe { Register::at(LLE_BASE, 0x04) },
            lle_status: unsafe { Register::at(LLE_BASE, 0x08) },
            lle_interrupt_enable: unsafe { Register::at(LLE_BASE, 0x0c) },
            lle_rx_to_tx_timing: unsafe { Register::at(LLE_BASE, 0x1c) },
            lle_mode: unsafe { Register::at(LLE_BASE, 0x50) },
            lle_operation_countdown: unsafe { Register::at(LLE_BASE, 0x64) },
            lle_tx_buffer: unsafe { Register::at(LLE_BASE, LLE_TX_BUFFER_OFFSET) },
            lle_rx_buffer: unsafe { Register::at(LLE_BASE, LLE_RX_BUFFER_OFFSET) },
        }
    }
}

/// Result of one hardware-chained connection exchange.
pub(crate) struct ExchangeResult<'a> {
    pub(crate) received: ReceivedPacket<'a>,
    pub(crate) transmitted: bool,
}

pub struct Radio {
    registers: RadioRegisters,
    access_address: u32,
    crc_init: u32,
    phy: Phy,
    digital_initialized: bool,
    frontend_calibrated: bool,
}

impl Radio {
    /// Creates the unique radio instance from the capability returned by
    /// [`crate::split`].
    pub fn new(_token: crate::RadioToken) -> Self {
        RF_DMA_LIMIT_DIAGNOSTICS[1].store(0, Ordering::Relaxed);
        RF_DMA_LIMIT_DIAGNOSTICS[2].store(0, Ordering::Relaxed);
        RF_DMA_LIMIT_DIAGNOSTICS[0].store(RF_DMA_LIMITS_MAGIC, Ordering::Release);
        Self {
            registers: unsafe { RadioRegisters::new() },
            access_address: ble_controller::link_layer::ADV_ACCESS_ADDRESS,
            crc_init: ble_controller::link_layer::ADV_CRC_INIT,
            phy: Phy::Le1M,
            digital_initialized: false,
            frontend_calibrated: false,
        }
    }

    /// Initializes the digital BLE engine and DMA plumbing.
    ///
    /// RF front-end calibration remains a separate, fallible milestone. TX is
    /// enabled only after `initialize()` completes calibration.
    pub fn init_digital(&mut self) {
        self.init_lle();
        self.init_baseband();
        self.digital_initialized = true;
        self.frontend_calibrated = false;
    }

    fn init_lle(&self) {
        let tx = core::ptr::addr_of_mut!(TX_BUFFER).cast::<u8>() as u32;
        let rx = core::ptr::addr_of_mut!(RX_BUFFER).cast::<u8>() as u32;

        // Both chips have distinct TX and RX DMA pointers, but CH585 moved
        // them from +0x70/+0x74 to +0x78/+0x7c.
        self.registers.lle_tx_buffer.write(tx);
        self.registers.lle_rx_buffer.write(rx);
        // Chip-specific LLE timing constants recovered from each public
        // archive.

        let timing = [
            (0x14, 0x8c),
            (0x1c, 0x76),
            (0x24, 0x8c),
            (0x2c, 0x6e),
            (0x34, 0x8c),
            (0x3c, 0x6e),
            (0x44, 0x8c),
            (0x4c, 0x76),
        ];
        for (offset, value) in timing {
            unsafe { Register::at(LLE_BASE, offset) }.write(value);
        }

        {
            // The BLE controller's LLE_DevInit clears the low five config
            // bits, then writes one to the half-word at +0x06. This differs
            // from the standalone RF-basic initialization sequence.
            self.registers.lle_config.modify(|value| value & !0x1f);
            unsafe { write_volatile((LLE_BASE + 0x06) as *mut u16, 1) };
            unsafe { Register::at(LLE_BASE, 0x54) }.write(0);
            self.registers.lle_status.write(u32::MAX);
            self.registers.lle_interrupt_enable.write(0x001f_000f);
        }
    }

    fn init_baseband(&mut self) {
        {
            self.registers
                .bb_config
                .modify(|value| (value | (1 << 23)) & !BB_MODE_MASK | BB_MODE_IDLE);
            self.registers.bb_interrupt_status.write(0x3ff);
            self.registers.bb_interrupt_enable.write(0x50);
            self.registers.bb_tx_control.modify(|value| {
                (value | (1 << 31)) & !BB_TX_POWER_MASK | (u32::from(TX_POWER_0_DBM) << 25)
            });
            // BB_DevInit derives this RFEND field from the selected PA code.
            // CH585's 0 dBm code (0x15) selects profile 1.
            unsafe { Register::at(RFEND_BASE, 0x5c) }
                .modify(|value| (value & !0x0700_0000) | 0x0100_0000);
            self.registers.bb_config_3c.write(0x0002_020c);
            self.registers
                .bb_receive_config_10
                .modify(|value| (value & !0x3f) | 0x0e);
        }
        self.registers.lle_mode.write(LLE_MODE_IDLE);
        self.set_access_address(self.access_address);
        self.set_crc_init(self.crc_init);
    }

    /// Initializes the complete digital and analog radio path.
    ///
    /// Calibration measurements are generated by this chip's RF analog block;
    /// no vendor firmware or precomputed per-die values are used.
    pub fn initialize(&mut self) -> Result<(), Error> {
        // The calibration state is order-sensitive: initialize the LLE,
        // reset/configure RFEND, and only then initialize the baseband. Doing
        // BB before RFEND produces stable but different per-die GA samples on
        // this board and changes the final RFEND compensation-table words.
        self.init_lle();
        let frontend =
            unsafe { crate::rf_frontend::FrontEnd::new(self.registers.lle_operation_countdown) };
        frontend.initialize_baseline();
        self.init_baseband();
        self.digital_initialized = true;
        self.frontend_calibrated = false;

        // Put the BB/LLE into this chip's analog-calibration state.

        {
            // RFIP_Calibration first shuts down any stale LLE operation, then
            // selects the CH585 LE-1M calibration profile and active BB bank.
            self.registers.lle_mode.modify(|value| value & !0x700);
            self.registers.lle_command.modify(|value| value | 0x08);
            for _ in 0..4 {
                core::hint::spin_loop();
            }
            self.registers.bb_config.modify(|value| value | (1 << 7));
            self.registers
                .bb_config
                .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_ACTIVE);
        }
        self.registers.lle_mode.write(LLE_MODE_CALIBRATION);

        let result = frontend.calibrate();

        // Always return to idle, including timeout and degenerate-measurement
        // paths, so a failed bring-up remains recoverable through the debugger.
        self.registers
            .bb_config
            .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_IDLE);

        self.registers.lle_status.write(LLE_STATUS_TIMING);
        frontend.leave_calibration();
        self.registers.lle_mode.write(LLE_MODE_IDLE);
        frontend.record_terminal(result.err());

        self.frontend_calibrated = result.is_ok();
        if result.is_ok() {
            // BLEB is PFIC IRQ20 on CH585. BLEL remains disabled until its
            // Rust handler is implemented; polling owns the LLE service
            // points while this IRQ handles the timing-critical BB capture.
            unsafe { qingke::pfic::enable_interrupt(20) };
        }
        result
    }

    fn stage_transmit(&mut self, channel: Channel, packet: &[u8]) -> Result<(), Error> {
        if !self.digital_initialized || !self.frontend_calibrated {
            return Err(Error::Hardware);
        }
        if packet.len() < 2 || packet.len() > RF_DMA_BUFFER_LEN {
            return Err(Error::InvalidPacket);
        }

        let tx = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFER) };
        encode_transmit_buffer(&mut tx.0, packet);

        self.set_channel(channel);
        self.registers
            .lle_tx_buffer
            .write(core::ptr::addr_of!(TX_BUFFER).cast::<u8>() as u32);
        Ok(())
    }

    fn stage_response(&self, packet: &[u8]) -> Result<u32, Error> {
        if packet.len() < 2 || packet.len() > RF_DMA_BUFFER_LEN {
            return Err(Error::InvalidPacket);
        }
        let tx = unsafe { &mut *core::ptr::addr_of_mut!(RESPONSE_BUFFER) };
        encode_transmit_buffer(&mut tx.0, packet);
        Ok(core::ptr::addr_of!(RESPONSE_BUFFER).cast::<u8>() as u32)
    }

    fn set_channel(&self, channel: Channel) {
        self.registers
            .bb_config
            .modify(|value| (value & !0x7f) | u32::from(channel.index()));

        self.configure_ch585_synth(channel, false);
    }

    fn set_receive_channel(&self, channel: Channel) {
        // Direct RX keeps the unmodified BLE channel index in BB_CONFIG[6:0].
        // RX direction is selected by RFEND; OR'ing bit 6 into a data channel
        // corrupts the dewhitening seed.
        self.registers
            .bb_config
            .modify(|value| (value & !0x7f) | u32::from(channel.index()));

        {
            self.configure_ch585_synth(channel, true);
            // The connected-Slave receive path clears both RFEND direction
            // controls after programming the synthesizer and before exposing
            // the BLE channel to the baseband. Standalone RFIP_SetRx leaves
            // bit 1 asserted, so these two operation classes cannot share the
            // synthesizer helper's final state.
            unsafe { Register::at(RFEND_BASE, 0x2c) }.modify(|value| value & !0x03);
        }
    }

    fn configure_ch585_synth(&self, channel: Channel, receive: bool) {
        // RFIP_DevSetChannel programs CH585's fractional synthesizer in kHz.
        // The current LE 1M receive profile follows RFIP_DevSetChannel's
        // non-0x10 mode branch and uses a 1 MHz low-side offset. The separate
        // 0x10 mode uses 2 MHz; selecting that value here was rejected by the
        // fixed-channel CRC hardware gate.
        let mut frequency_khz = u32::from(channel.frequency_mhz()) * 1_000;
        if receive {
            frequency_khz -= 1_000;
        }
        unsafe { Register::at(RFEND_BASE, 0x2c) }.modify(|value| value | 0x02);
        let integer = (frequency_khz / 64_000) & 0x1f;
        let fraction = (((frequency_khz % 64_000) << 10) / 250) & 0x0003_ffff;
        unsafe { Register::at(RFEND_BASE, 0x44) }
            .modify(|value| (value & 0xfe0c_0000) | (integer << 20) | fraction);
    }

    fn prepare_common_operation(&self, mode: u32) {
        unsafe {
            write_volatile(
                (LLE_BASE + 0x06) as *mut u16,
                if mode == LLE_MODE_TX { 0x50 } else { 1 },
            )
        };
        self.registers
            .bb_tx_control
            .modify(|value| (value & !0x03) | 0x01);
        self.registers
            .bb_config
            .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_ACTIVE);
        unsafe { Register::at(RFEND_BASE, 0x08) }.modify(|value| value | 0x0033_0000);
        self.registers.lle_mode.write(mode);
    }

    fn prepare_receive_operation(&self) {
        unsafe { write_volatile((LLE_BASE + 0x06) as *mut u16, 1) };
        self.registers
            .bb_config
            .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_ACTIVE);
        unsafe { Register::at(RFEND_BASE, 0x08) }.modify(|value| value | 0x0033_0000);
        self.registers.lle_mode.write(LLE_MODE_RX);
    }

    fn wait_for_settle_countdown(&self) -> Result<(), Error> {
        for _ in 0..MAX_OPERATION_POLLS {
            if self.registers.lle_operation_countdown.read() == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }

    fn configure_transmit_phy_1m(&self, packet_len: usize) {
        // The two baseband generations select LE 1M differently. CH582 uses
        // BB_CONFIG[13:12]. CH585 uses bit 7 as the 2M selector, so LE 1M
        // must explicitly clear it.

        self.registers.bb_config.modify(|value| value & !(1 << 7));
        self.registers
            .lle_interrupt_enable
            .modify(|value| value & !LLE_STATUS_TIMING);
        self.registers.lle_status.write(LLE_STATUS_TIMING);
        self.registers
            .lle_operation_countdown
            .write(tx_1m_countdown(packet_len));
    }

    fn configure_receive_phy_1m(&self) {
        {
            // BLE_SetPHYRxMode(LE1M) clears the 2M selector, installs 0x132
            // in BB+0x18[9:0], and writes 0x7f to its top byte.
            self.registers.bb_config.modify(|value| value & !(1 << 7));
            self.registers
                .bb_receive_config_18
                .modify(|value| (value & !0xff00_03ff) | 0x7f00_0132);
        }
    }

    fn continue_tx_to_rx_1m(&self, _channel: Channel) {
        // In a chained advertising operation, status bit 3 is the service
        // point between the initial TX and RX legs. The CH582 implementation
        // acknowledges that service point, restores BB_TX_CONTROL to the
        // staged state, selects the RX PHY, clears the LLE 0x2000 timing
        // source and arms the LE 1M transition countdown (0x196).
        self.registers.lle_status.write(LLE_STATUS_TX_TO_RX);
        self.registers
            .bb_tx_control
            .modify(|value| (value & !0x03) | 0x01);

        self.configure_ch585_synth(_channel, true);
        // Unlike command-1 direct RX, the automatic advertising transition
        // keeps the TX channel encoding. The LLE owns the direction change.
        self.configure_receive_phy_1m();
        self.registers.lle_status.write(LLE_STATUS_TIMING);
        self.registers
            .lle_operation_countdown
            .write(LLE_1M_TRANSITION_COUNTDOWN);
    }

    fn release_receive_to_transmit_1m(
        &self,
        _channel: Channel,
        transmit_address: u32,
        transmit_len: usize,
        received_len: u8,
    ) {
        // This is the register-level equivalent of the vendor LLE bit-2
        // service followed by the `ll_tx_wait_finish(3, ..)` release path.
        // The response is already resident in SRAM; no protocol construction
        // is allowed inside this timing-critical operation.
        self.registers.lle_tx_buffer.write(transmit_address);
        self.registers
            .lle_rx_to_tx_timing
            .write(rx_to_tx_timing(received_len));
        // Keep the CH585 response release order; the payload is already staged.
        self.registers.lle_status.write(LLE_STATUS_RX_TO_TX);
        self.registers
            .bb_tx_control
            .modify(|value| (value & !0x03) | 0x01);

        self.configure_ch585_synth(_channel, false);
        self.configure_transmit_phy_1m(transmit_len);
        self.registers
            .bb_config
            .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_ACTIVE | (1 << 23));
        unsafe { Register::at(RFEND_BASE, 0x08) }.modify(|value| value | 0x0033_0000);
        self.registers.lle_mode.write(LLE_MODE_TX);
        self.registers.bb_tx_control.modify(|value| value & !0x03);
    }

    fn start_transmit_operation(&self) {
        // CH582 stages TX with BB_TX_CONTROL[1:0] = 01, but the actual LLE
        // command is launched with those bits cleared. Keeping 01 still
        // produces an LLE completion indication while suppressing a valid
        // over-air packet.
        self.registers.bb_config.modify(|value| value | (1 << 23));
        self.registers.bb_tx_control.modify(|value| value & !0x03);
        self.registers.lle_command.write(2);
    }

    fn shut_down_operation(&self) {
        RX_CAPTURE_COUNTDOWN.store(0, Ordering::Release);

        {
            self.registers.lle_mode.modify(|value| value & !0x10);
            for _ in 0..4 {
                core::hint::spin_loop();
            }
            self.registers.lle_mode.modify(|value| value & !0x700);
            self.registers.lle_command.modify(|value| value | 0x08);
        }
        for _ in 0..4 {
            core::hint::spin_loop();
        }
        self.registers
            .bb_config
            .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_IDLE);
        unsafe { Register::at(RFEND_BASE, 0x08) }.modify(|value| value & 0xffcd_ffff);
        self.registers.lle_mode.write(LLE_MODE_IDLE);
    }

    fn wait_for_operation(&self, completion_mask: u32) -> Result<(), Error> {
        for _ in 0..MAX_OPERATION_POLLS {
            if self.registers.lle_status.read() & completion_mask != 0 {
                return Ok(());
            }
            if self.registers.lle_operation_countdown.read() == 0 {
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }

    fn clear_bb_interrupt_status(&self) {
        {
            let observed = self.registers.bb_interrupt_status.read();
            self.registers
                .bb_interrupt_status
                .write(observed & BB_INTERRUPT_STATUS_MASK);
        }
    }

    fn clear_lle_status(&self) {
        // CH582 exposes the operation flags in the low half-word. CH585 adds
        // capture/timing sources in the high half-word; carrying one of those
        // W1C flags from advertising into a connection event can make the next
        // operation observe stale controller state.

        self.registers.lle_status.write(u32::MAX);
    }

    fn wait_for_polled_receive_operation(
        &self,
        software_receive_deadline: embassy_time::Instant,
    ) -> Result<(), Error> {
        let mut capture_started = false;
        for _ in 0..MAX_OPERATION_POLLS {
            let bb_status = self.registers.bb_interrupt_status.read();
            if bb_status & (1 << 6) != 0 {
                // Standalone scanning owns this service point by polling.
                // Re-arm the packet-duration countdown exactly as BLEB does,
                // but do not let the interrupt race this loop and clear the
                // only indication before it is observed.

                self.registers
                    .bb_interrupt_status
                    .write(rx_status::capture_ack(bb_status));
                self.registers.lle_status.write(LLE_STATUS_TIMING);
                self.registers
                    .lle_operation_countdown
                    .write(rx_1m_capture_countdown(LEGACY_ADV_RX_MAX_LEN));
                capture_started = true;
            }
            if capture_started
                && self.registers.lle_status.read()
                    & (LLE_STATUS_RX_COMPLETE | LLE_STATUS_RX_CAPTURE_COMPLETE)
                    != 0
            {
                return Ok(());
            }
            if self.registers.lle_operation_countdown.read() == 0 {
                return Err(Error::Timeout);
            }
            // In direct RX the hardware countdown does not start until the
            // demodulator sees a packet. Keep the caller's scan window as a
            // wall-clock bound instead of letting the CPU-speed-dependent poll
            // fallback extend a short window. This path never transmits an IFS
            // response, so there is no turnaround phase to preserve here.
            if !capture_started && embassy_time::Instant::now() >= software_receive_deadline {
                return Err(Error::Timeout);
            }
            core::hint::spin_loop();
        }
        Err(Error::Timeout)
    }

    fn transmit_blocking(&mut self, channel: Channel, packet: &[u8]) -> Result<(), Error> {
        self.stage_transmit(channel, packet)?;
        self.prepare_common_operation(LLE_MODE_TX);
        self.registers.lle_config.modify(|value| value & !0x01);
        self.clear_lle_status();
        self.clear_bb_interrupt_status();
        // The first countdown is the RF/BB settle window. The vendor
        // behavior waits for it to expire before configuring the actual LE
        // 1M packet-duration countdown and launching command 2.
        self.registers.lle_operation_countdown.write(0xa0);
        self.wait_for_settle_countdown()?;
        self.configure_transmit_phy_1m(packet.len());
        self.start_transmit_operation();
        // LLE status bit 1 is TX completion (the vendor IRQ handler clears it
        // with a write-one-to-clear operation).
        let result = self.wait_for_operation(1 << 1);
        self.shut_down_operation();
        result
    }

    fn receive_blocking<'a>(
        &mut self,
        channel: Channel,
        timeout: embassy_time::Duration,
        output: &'a mut [u8],
    ) -> Result<ReceivedPacket<'a>, Error> {
        if !self.digital_initialized || !self.frontend_calibrated {
            return Err(Error::Hardware);
        }
        if output.is_empty() {
            return Err(Error::InvalidPacket);
        }

        let rx_address = core::ptr::addr_of_mut!(RX_BUFFER).cast::<u8>() as u32;
        let rx = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) };
        rx.0.fill(0xff);
        self.registers.lle_rx_buffer.write(rx_address);
        self.set_receive_channel(channel);
        self.registers.lle_config.modify(|value| value & !0x01);
        self.clear_lle_status();
        self.clear_bb_interrupt_status();
        self.configure_receive_phy_1m();

        // The direct scanner RX path polls the same BB capture point serviced
        // by BLEB in connection/advertising exchanges. It must have exactly
        // one owner: otherwise the IRQ can clear bit 6 between the poll and
        // its capture-countdown update. Keep the already validated BLEB path
        // unchanged for all IFS-critical exchanges.
        unsafe {
            qingke::pfic::disable_interrupt(20);
            qingke::pfic::unpend_interrupt(20);
        }

        // The hardware countdown uses quarter-microsecond units in the direct
        // RX path. Saturation keeps malformed host timeouts recoverable.
        let ticks = timeout
            .as_micros()
            .saturating_mul(4)
            .clamp(1, u32::MAX as u64) as u32;
        self.registers.lle_operation_countdown.write(ticks);
        self.prepare_receive_operation();
        let software_receive_deadline = embassy_time::Instant::now() + timeout;
        self.registers.lle_command.write(1);
        let wait = self.wait_for_polled_receive_operation(software_receive_deadline);

        let status = self.registers.bb_interrupt_status.read();
        let crc_ok = status & (1 << 5) == 0;
        let copy_len = copy_received_packet(
            &rx.0,
            output,
            self.access_address == ble_controller::link_layer::ADV_ACCESS_ADDRESS,
        );
        let rssi_dbm = -((self.registers.bb_rssi_status.read() >> 15) as u8 as i8);
        self.shut_down_operation();
        unsafe {
            qingke::pfic::unpend_interrupt(20);
            qingke::pfic::enable_interrupt(20);
        }

        wait?;
        if copy_len == 0 {
            return Err(Error::Timeout);
        }
        Ok(ReceivedPacket {
            payload: &output[..copy_len],
            rssi_dbm,
            crc_ok,
            timestamp_us: embassy_time::Instant::now().as_micros(),
        })
    }

    /// Runs one bounded Peripheral RX-to-TX exchange with a pre-staged owner.
    pub(crate) async fn exchange<'a>(
        &mut self,
        channel: Channel,
        deadline: embassy_time::Instant,
        timeout: embassy_time::Duration,
        transmit: &[u8],
        output: &'a mut [u8],
    ) -> Result<ExchangeResult<'a>, Error> {
        if output.len() < CONNECTION_EVENT_RX_BUFFER_LEN {
            return Err(Error::InvalidPacket);
        }
        self.stage_transmit(channel, transmit)?;
        let rx = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) };
        rx.0.fill(0xff);
        self.registers.lle_rx_buffer.write(rx.0.as_mut_ptr() as u32);
        self.set_receive_channel(channel);
        self.prepare_common_operation(LLE_MODE_RX);
        self.registers.lle_config.modify(|value| value | 0x01);
        self.clear_lle_status();
        self.clear_bb_interrupt_status();
        self.configure_receive_phy_1m();
        RX_CAPTURE_COUNTDOWN.store(
            rx_1m_capture_countdown(CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN),
            Ordering::Release,
        );
        let ticks = timeout
            .as_micros()
            .saturating_mul(4)
            .clamp(1, u64::from(u32::MAX)) as u32;
        self.registers.lle_operation_countdown.write(ticks);
        self.registers.lle_command.write(1);
        let receive_deadline = core::cmp::min(embassy_time::Instant::now() + timeout, deadline);
        let mut timestamp_us = None;
        let mut turnaround = false;
        let mut transmitted = false;
        // A second bound covers a stuck TX state without cutting a normal IFS.
        let operation_deadline = receive_deadline + embassy_time::Duration::from_micros(1_000);
        while embassy_time::Instant::now() < operation_deadline {
            let status = self.registers.lle_status.read();
            if status & LLE_STATUS_RX_TO_TX != 0 && !turnaround {
                let length = read_dma_byte(&rx.0, 1);
                timestamp_us = Some(embassy_time::Instant::now().as_micros());
                if self.registers.bb_interrupt_status.read() & (1 << 5) != 0
                    || length as usize > CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN
                {
                    break;
                }
                // Preserve the ordinary staged SN/NESN policy. This release
                // sequence matches the Rust single-PDU CH585 register path.
                self.registers
                    .lle_rx_to_tx_timing
                    .write(rx_to_tx_timing(length));
                self.registers.lle_status.write(LLE_STATUS_RX_COMPLETE);
                self.registers.lle_status.write(LLE_STATUS_RX_TO_TX);
                self.registers
                    .bb_tx_control
                    .modify(|value| (value & !0x03) | 0x01);
                self.configure_transmit_phy_1m(transmit.len());
                self.configure_ch585_synth(channel, false);
                self.registers
                    .bb_config
                    .modify(|value| (value & !BB_MODE_MASK) | BB_MODE_ACTIVE | (1 << 23));
                unsafe { Register::at(RFEND_BASE, 0x08) }.modify(|value| value | 0x0033_0000);
                self.registers.lle_mode.write(LLE_MODE_TX);
                self.registers.bb_tx_control.modify(|value| value & !0x03);
                turnaround = true;
                // Only a fresh status read may establish response completion.
                continue;
            }
            if turnaround && status & LLE_STATUS_RX_COMPLETE != 0 {
                self.registers.lle_status.write(LLE_STATUS_RX_COMPLETE);
                transmitted = true;
                break;
            }
            if self.registers.lle_operation_countdown.read() == 0 {
                break;
            }
            if !turnaround && embassy_time::Instant::now() >= receive_deadline {
                break;
            }
            core::hint::spin_loop();
        }
        let crc_ok = self.registers.bb_interrupt_status.read() & (1 << 5) == 0;
        let len = copy_received_packet(&rx.0, output, false);
        let rssi_dbm = -((self.registers.bb_rssi_status.read() >> 15) as u8 as i8);
        self.shut_down_operation();
        let timestamp_us = timestamp_us.ok_or(Error::Timeout)?;
        if len < 2 {
            return Err(Error::InvalidPacket);
        }
        Ok(ExchangeResult {
            received: ReceivedPacket {
                payload: &output[..len],
                rssi_dbm,
                crc_ok,
                timestamp_us,
            },
            transmitted,
        })
    }

    /// Sends one advertisement, then receives SCAN_REQ or CONNECT_IND.
    pub(crate) async fn advertising_exchange<'a>(
        &mut self,
        channel: Channel,
        timeout: embassy_time::Duration,
        transmit: &[u8],
        scan_response: &[u8],
        advertiser_address: [u8; 6],
        output: &'a mut [u8],
    ) -> Result<ReceivedPacket<'a>, Error> {
        self.stage_transmit(channel, transmit)?;
        let response_address = self.stage_response(scan_response)?;
        let rx = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) };
        rx.0.fill(0xff);
        self.registers.lle_rx_buffer.write(rx.0.as_mut_ptr() as u32);
        self.prepare_common_operation(LLE_MODE_TX);
        self.registers.lle_config.modify(|value| value | 1);
        self.clear_lle_status();
        self.clear_bb_interrupt_status();
        self.registers.lle_operation_countdown.write(0xa0);
        self.wait_for_settle_countdown()?;
        self.configure_transmit_phy_1m(transmit.len());
        RX_CAPTURE_COUNTDOWN.store(
            rx_1m_capture_countdown(LEGACY_ADV_RX_MAX_LEN),
            Ordering::Release,
        );
        CAPTURE_SEEN.store(false, Ordering::Release);
        self.start_transmit_operation();
        let mut deadline =
            embassy_time::Instant::now() + embassy_time::Duration::from_micros(2_000);
        let mut armed = false;
        let mut responding = false;
        let mut buffered_end = None;
        let mut received = None;
        while embassy_time::Instant::now() < deadline {
            let status = self.registers.lle_status.read();
            if !armed && status & (LLE_STATUS_TX_TO_RX | LLE_STATUS_TX_COMPLETE) != 0 {
                self.continue_tx_to_rx_1m(channel);
                armed = true;
                deadline = embassy_time::Instant::now()
                    + embassy_time::Duration::from_micros(
                        timeout.as_micros().max(LEGACY_ADV_RESPONSE_WINDOW_US),
                    );
            }
            let pdu_type = read_dma_byte(&rx.0, 0) & 0x0f;
            let length = read_dma_byte(&rx.0, 1) & 0x3f;
            if armed
                && buffered_end.is_none()
                && legacy_request_payload_len_is_valid(pdu_type, length as usize)
            {
                buffered_end = Some(
                    embassy_time::Instant::now()
                        + embassy_time::Duration::from_micros((u64::from(length) + 3) * 8),
                );
            }
            let buffered_scan = pdu_type == 3
                && buffered_end.is_some_and(|end| embassy_time::Instant::now() >= end);
            // Observe the IRQ flag before sampling LLE status. An older status
            // word can contain the capture-start bit that the IRQ just cleared.
            // Pairing that old word with a new flag exposes unfinished DMA.
            let captured = CAPTURE_SEEN.load(Ordering::Acquire)
                && (self.registers.lle_status.read()
                    & (LLE_STATUS_RX_COMPLETE | LLE_STATUS_RX_CAPTURE_COMPLETE)
                    != 0
                    || self.registers.lle_operation_countdown.read() == 0);
            if armed
                && !responding
                && (status & LLE_STATUS_RX_TO_TX != 0 || captured || buffered_scan)
            {
                let crc_ok = self.registers.bb_interrupt_status.read() & (1 << 5) == 0;
                let len = copy_received_packet(&rx.0, output, true);
                if len >= 2 && read_dma_byte(&rx.0, 0) != 0xff {
                    received = Some((len, crc_ok, embassy_time::Instant::now().as_micros()));
                    let address = ble_controller::advertising::DeviceAddress {
                        bytes: advertiser_address,
                        kind: if transmit[0] & (1 << 6) != 0 {
                            ble_controller::advertising::AddressKind::Random
                        } else {
                            ble_controller::advertising::AddressKind::Public
                        },
                    };
                    if crc_ok
                        && ble_controller::advertising::is_scan_request(&output[..len], address)
                    {
                        self.release_receive_to_transmit_1m(
                            channel,
                            response_address,
                            scan_response.len(),
                            length,
                        );
                        responding = true;
                        deadline = embassy_time::Instant::now()
                            + embassy_time::Duration::from_micros(1_000);
                        continue;
                    }
                    break;
                }
            }
            if responding && status & (LLE_STATUS_RX_COMPLETE | LLE_STATUS_TX_COMPLETE) != 0 {
                break;
            }
            core::hint::spin_loop();
        }
        let rssi_dbm = -((self.registers.bb_rssi_status.read() >> 15) as u8 as i8);
        self.shut_down_operation();
        let (len, crc_ok, timestamp_us) = received.ok_or(Error::Timeout)?;
        Ok(ReceivedPacket {
            payload: &output[..len],
            crc_ok,
            timestamp_us,
            rssi_dbm,
        })
    }
}

/// CH585 baseband interrupt. Keep this handler bounded: it only services the
/// timing-critical receive-capture point and clears other enabled BB sources.
/// Protocol parsing and all host work remain in the Embassy task.
///
/// Owned by the HAL BLE token consumed by this backend.
#[qingke_rt::interrupt]
fn BLEB() {
    let bb_status = unsafe { Register::at(BB_BASE, 0x38) };
    let lle_status = unsafe { Register::at(LLE_BASE, 0x08) };
    let lle_countdown = unsafe { Register::at(LLE_BASE, 0x64) };
    let status = bb_status.read();

    if status & (1 << 6) != 0 {
        bb_status.write(rx_status::capture_ack(status));
        let countdown = RX_CAPTURE_COUNTDOWN.load(Ordering::Acquire);
        if countdown != 0 {
            lle_status.write(LLE_STATUS_TIMING);
            lle_countdown.write(countdown);
            CAPTURE_SEEN.store(true, Ordering::Release);
        }
    }

    // Clear the remaining enabled sources using this generation's semantics:
    // CH582 clears by writing the observed word with those bits low; CH585's
    // BB status is W1C and its public handler writes the asserted mask.
    let remaining = bb_status.read();

    bb_status.write(remaining & 0x190);
}

#[inline(always)]
fn read_dma_byte(buffer: &[u8; RF_DMA_BUFFER_LEN], index: usize) -> u8 {
    // The baseband writes this SRAM behind Rust's view of the allocation.
    // Ordinary slice reads may be cached or reordered across MMIO polling.
    unsafe { read_volatile(buffer.as_ptr().add(index)) }
}

fn copy_received_packet(
    buffer: &[u8; RF_DMA_BUFFER_LEN],
    output: &mut [u8],
    advertising: bool,
) -> usize {
    let payload_len = if advertising {
        usize::from(read_dma_byte(buffer, 1) & 0x3f)
    } else {
        usize::from(read_dma_byte(buffer, 1))
    };
    let wire_len = match classify_received_length(advertising, payload_len) {
        ReceiveLengthPolicy::Accepted { wire_len } => wire_len,
        ReceiveLengthPolicy::PhysicalDmaOverflow => {
            RF_DMA_LIMIT_DIAGNOSTICS[1].fetch_add(1, Ordering::Relaxed);
            return 0;
        }
        ReceiveLengthPolicy::UnsupportedCapability => {
            RF_DMA_LIMIT_DIAGNOSTICS[2].fetch_add(1, Ordering::Relaxed);
            return 0;
        }
    };
    if output.len() < wire_len {
        return 0;
    }
    let copy_len = wire_len;
    for (index, byte) in output[..copy_len].iter_mut().enumerate() {
        *byte = read_dma_byte(buffer, index);
    }
    copy_len
}

impl BleRadio for Radio {
    async fn transmit(&mut self, channel: Channel, packet: &[u8]) -> Result<(), Error> {
        self.transmit_blocking(channel, packet)
    }

    async fn receive<'a>(
        &mut self,
        channel: Channel,
        timeout: embassy_time::Duration,
        buffer: &'a mut [u8],
    ) -> Result<ReceivedPacket<'a>, Error> {
        self.receive_blocking(channel, timeout, buffer)
    }

    fn set_access_address(&mut self, access_address: u32) {
        self.access_address = access_address;
        self.registers.bb_tx_access_address.write(access_address);
    }

    fn set_crc_init(&mut self, crc_init: u32) {
        self.crc_init = crc_init & 0x00ff_ffff;
        self.registers.bb_tx_crc_init.write(self.crc_init);
    }

    fn set_phy(&mut self, phy: Phy) -> Result<(), Error> {
        // The first controller milestone supports only uncoded LE 1M. Do not
        // select register modes for PHYs that have not been validated.
        if phy != Phy::Le1M {
            return Err(Error::Hardware);
        }
        self.phy = phy;
        Ok(())
    }
}

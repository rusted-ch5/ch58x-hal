//! Blocking UART0..UART3 support.
//!
//! The CH58x UARTs have an eight-byte FIFO and a two-stage baud divider.  This
//! module owns the PAC peripheral and both pins, resets stale bootloader state,
//! and configures the selected default or remapped pin pair.

use core::fmt;

use crate::gpio::{Drive, Flex, GpioPin, Level, Pin, Pull};
use crate::{pac, sysctl};

const FIFO_CAPACITY: usize = 8;

/// Number of data bits in one UART frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DataBits {
    Five = 0,
    Six = 1,
    Seven = 2,
    Eight = 3,
}

/// Number of stop bits in one UART frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StopBits {
    One,
    Two,
}

/// UART parity mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Parity {
    None,
    Odd,
    Even,
    Mark,
    Space,
}

/// UART configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
    /// Maximum accepted absolute baud-rate error in parts per million.
    pub max_error_ppm: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115_200,
            data_bits: DataBits::Eight,
            stop_bits: StopBits::One,
            parity: Parity::None,
            max_error_ppm: 20_000,
        }
    }
}

/// UART construction failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    BaudrateZero,
    BaudrateOutOfRange,
    BaudrateErrorTooHigh { actual: u32, error_ppm: u32 },
    MismatchedPinRemap,
}

/// UART receive error reported by the line-status register.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Overrun,
    Parity,
    Framing,
    Break,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Overrun => embedded_io::ErrorKind::Other,
            Self::Parity => embedded_io::ErrorKind::InvalidData,
            Self::Framing | Self::Break => embedded_io::ErrorKind::InvalidData,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BaudDividers {
    predivider: u8,
    latch: u16,
    actual: u32,
    error_ppm: u32,
}

/// Find the least-error `(DIV, DL)` pair. DIV encodes 128 as zero.
const fn select_baud(clock_hz: u32, baudrate: u32) -> Option<BaudDividers> {
    if clock_hz == 0 || baudrate == 0 {
        return None;
    }

    let mut predivider = 1u32;
    let mut best: Option<BaudDividers> = None;
    while predivider <= 128 {
        let denominator = 8u64 * predivider as u64 * baudrate as u64;
        let rounded_latch = (clock_hz as u64 + denominator / 2) / denominator;
        if rounded_latch >= 1 && rounded_latch <= u16::MAX as u64 {
            let actual = (clock_hz as u64 / (8 * predivider as u64 * rounded_latch)) as u32;
            let difference = actual.abs_diff(baudrate);
            let error_ppm = ((difference as u64 * 1_000_000) / baudrate as u64) as u32;
            let candidate = BaudDividers {
                predivider: if predivider == 128 {
                    0
                } else {
                    predivider as u8
                },
                latch: rounded_latch as u16,
                actual,
                error_ppm,
            };
            if best.is_none() || error_ppm < best.unwrap().error_ppm {
                best = Some(candidate);
            }
        }
        predivider += 1;
    }
    best
}

// Compile-time checks keep the clean-room divider implementation anchored to
// the documented baud = HCLK / (8 * DIV * DL) equation.
const _: () = {
    let standard = select_baud(60_000_000, 115_200).unwrap();
    assert!(standard.actual == 115_384);
    assert!(standard.error_ppm < 2_000);
    let slow = select_baud(60_000_000, 300).unwrap();
    assert!(slow.error_ppm < 2_000);
    assert!(select_baud(60_000_000, 0).is_none());
};

mod sealed {
    use super::*;

    pub trait Instance {
        const INDEX: usize;

        fn regs() -> &'static pac::uart0::RegisterBlock;
    }

    pub trait TxPin<T: Instance>: GpioPin {
        const REMAPPED: bool;
    }

    pub trait RxPin<T: Instance>: GpioPin {
        const REMAPPED: bool;
    }
}

/// A CH58x UART peripheral instance.
pub trait Instance: sealed::Instance + Send + 'static {}

/// A valid transmit pin for UART instance `T`.
pub trait TxPin<T: Instance>: sealed::TxPin<T> {}

/// A valid receive pin for UART instance `T`.
pub trait RxPin<T: Instance>: sealed::RxPin<T> {}

macro_rules! impl_instance {
    ($peripheral:ty, $index:expr) => {
        impl sealed::Instance for $peripheral {
            const INDEX: usize = $index;

            fn regs() -> &'static pac::uart0::RegisterBlock {
                unsafe { &*<$peripheral>::PTR }
            }
        }

        impl Instance for $peripheral {}
    };
}

impl_instance!(pac::UART0, 0);
impl_instance!(pac::UART1, 1);
impl_instance!(pac::UART2, 2);
impl_instance!(pac::UART3, 3);

macro_rules! impl_pin {
    ($trait_name:ident, $peripheral:ty, $pin:ty, $remapped:expr) => {
        impl sealed::$trait_name<$peripheral> for $pin {
            const REMAPPED: bool = $remapped;
        }
        impl $trait_name<$peripheral> for $pin {}
    };
}

impl_pin!(TxPin, pac::UART0, Pin<1, 7>, false);
impl_pin!(RxPin, pac::UART0, Pin<1, 4>, false);
impl_pin!(TxPin, pac::UART0, Pin<0, 14>, true);
impl_pin!(RxPin, pac::UART0, Pin<0, 15>, true);

impl_pin!(TxPin, pac::UART1, Pin<0, 9>, false);
impl_pin!(RxPin, pac::UART1, Pin<0, 8>, false);
impl_pin!(TxPin, pac::UART1, Pin<1, 13>, true);
impl_pin!(RxPin, pac::UART1, Pin<1, 12>, true);

impl_pin!(TxPin, pac::UART2, Pin<0, 7>, false);
impl_pin!(RxPin, pac::UART2, Pin<0, 6>, false);
impl_pin!(TxPin, pac::UART2, Pin<1, 23>, true);
impl_pin!(RxPin, pac::UART2, Pin<1, 22>, true);

impl_pin!(TxPin, pac::UART3, Pin<0, 5>, false);
impl_pin!(RxPin, pac::UART3, Pin<0, 4>, false);
impl_pin!(TxPin, pac::UART3, Pin<1, 21>, true);
impl_pin!(RxPin, pac::UART3, Pin<1, 20>, true);

fn configure_remap<T: Instance>(remapped: bool) {
    let gpioctl = unsafe { &*pac::GPIOCTL::PTR };
    let mask = 1u16 << (4 + T::INDEX);
    critical_section::with(|_| unsafe {
        gpioctl.pin_alternate().modify(|r, w| {
            let bits = if remapped {
                r.bits() | mask
            } else {
                r.bits() & !mask
            };
            w.bits(bits)
        });
    });
}

fn configure<T: Instance>(config: Config) -> Result<BaudDividers, ConfigError> {
    if config.baudrate == 0 {
        return Err(ConfigError::BaudrateZero);
    }
    let baud =
        select_baud(sysctl::hclk_hz(), config.baudrate).ok_or(ConfigError::BaudrateOutOfRange)?;
    if baud.error_ppm > config.max_error_ppm {
        return Err(ConfigError::BaudrateErrorTooHigh {
            actual: baud.actual,
            error_ppm: baud.error_ppm,
        });
    }

    let parity_bits = match config.parity {
        Parity::None => 0,
        Parity::Odd => 1 << 3,
        Parity::Even => (1 << 3) | (1 << 4),
        Parity::Mark => (1 << 3) | (2 << 4),
        Parity::Space => (1 << 3) | (3 << 4),
    };
    let line_control = config.data_bits as u8
        | if config.stop_bits == StopBits::Two {
            1 << 2
        } else {
            0
        }
        | parity_bits;

    let regs = T::regs();
    regs.ier().write(|w| w.reset().set_bit());
    regs.fcr().write(|w| {
        w.fifo_en()
            .set_bit()
            .rx_fifo_clr()
            .set_bit()
            .tx_fifo_clr()
            .set_bit()
            .fifo_trig()
            .variant(0)
    });
    regs.lcr().write(|w| unsafe { w.bits(line_control) });
    regs.div().write(|w| unsafe { w.bits(baud.predivider) });
    regs.dl().write(|w| unsafe { w.bits(baud.latch) });
    // TXD_EN is independent of the transmit-empty interrupt.
    regs.ier().write(|w| w.txd_en().set_bit());
    regs.mcr().write(|w| w.out2__rb_mcr_int_oe().set_bit());
    Ok(baud)
}

/// Owned full-duplex UART.
pub struct Uart<T: Instance, TX: TxPin<T>, RX: RxPin<T>> {
    _peripheral: T,
    _tx: Flex<TX>,
    _rx: Flex<RX>,
    actual_baudrate: u32,
}

impl<T, TX, RX> Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    pub fn new(peripheral: T, tx: TX, rx: RX, config: Config) -> Result<Self, ConfigError> {
        if TX::REMAPPED != RX::REMAPPED {
            return Err(ConfigError::MismatchedPinRemap);
        }

        let mut tx = Flex::new(tx);
        tx.set_level(Level::High);
        tx.set_as_output(Drive::MilliAmps5);
        let mut rx = Flex::new(rx);
        rx.set_as_input(Pull::Up);
        configure_remap::<T>(TX::REMAPPED);
        let baud = configure::<T>(config)?;

        Ok(Self {
            _peripheral: peripheral,
            _tx: tx,
            _rx: rx,
            actual_baudrate: baud.actual,
        })
    }

    /// Actual baud rate produced by the selected integer dividers.
    pub fn actual_baudrate(&self) -> u32 {
        self.actual_baudrate
    }

    fn receive_byte() -> Result<Option<u8>, Error> {
        let regs = T::regs();
        let status = regs.lsr().read();
        let error = if status.over_err().bit_is_set() {
            Some(Error::Overrun)
        } else if status.par_err().bit_is_set() {
            Some(Error::Parity)
        } else if status.frame_err().bit_is_set() {
            Some(Error::Framing)
        } else if status.break_err().bit_is_set() {
            Some(Error::Break)
        } else {
            None
        };
        if let Some(error) = error {
            // Drop the bad FIFO head so the same read-to-clear status cannot
            // make every following operation fail forever.
            if status.data_rdy().bit_is_set() {
                let _ = regs.rbr().read().bits();
            }
            return Err(error);
        }
        if status.data_rdy().bit_is_set() {
            Ok(Some(regs.rbr().read().bits()))
        } else {
            Ok(None)
        }
    }

    fn read_available(buffer: &mut [u8]) -> Result<usize, Error> {
        let mut count = 0;
        while count < buffer.len() {
            match Self::receive_byte()? {
                Some(byte) => {
                    buffer[count] = byte;
                    count += 1;
                }
                None => break,
            }
        }
        Ok(count)
    }

    /// Blocking read which returns after at least one byte is available.
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if buffer.is_empty() {
            return Ok(0);
        }
        loop {
            let count = Self::read_available(buffer)?;
            if count != 0 {
                return Ok(count);
            }
        }
    }

    /// Blocking write of the complete buffer.
    pub fn blocking_write_all(&mut self, buffer: &[u8]) -> Result<(), Error> {
        for byte in buffer {
            while usize::from(T::regs().tfc().read().bits()) >= FIFO_CAPACITY {}
            T::regs().thr().write(|w| unsafe { w.bits(*byte) });
        }
        Ok(())
    }

    /// Block until both the FIFO and shift register are empty.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        while T::regs().lsr().read().tx_all_emp().bit_is_clear() {}
        Ok(())
    }
}

impl<T, TX, RX> Drop for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    fn drop(&mut self) {
        T::regs().ier().write(|w| unsafe { w.bits(0) });
    }
}

impl<T, TX, RX> embedded_io::ErrorType for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    type Error = Error;
}

impl<T, TX, RX> embedded_io::Read for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        self.blocking_read(buffer)
    }
}

impl<T, TX, RX> embedded_io::Write for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    fn write(&mut self, buffer: &[u8]) -> Result<usize, Self::Error> {
        self.blocking_write_all(buffer)?;
        Ok(buffer.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.blocking_flush()
    }
}

impl<T, TX, RX> fmt::Write for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    fn write_str(&mut self, string: &str) -> fmt::Result {
        self.blocking_write_all(string.as_bytes())
            .map_err(|_| fmt::Error)
    }
}

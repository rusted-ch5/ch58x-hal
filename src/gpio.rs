//! Owned GPIO pins for CH581/CH582/CH583/CH584/CH585.

use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

use crate::pac;

/// GPIO logic level.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Level {
    Low,
    High,
}

/// Input bias configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Pull {
    None,
    Up,
    Down,
}

/// Push-pull output drive strength.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Drive {
    #[default]
    MilliAmps5,
    MilliAmps20,
}

/// One uniquely-owned GPIO pin.
pub struct Pin<const PORT: u8, const N: u8> {
    _not_sendable_by_construction: PhantomData<*mut ()>,
}

impl<const PORT: u8, const N: u8> Pin<PORT, N> {
    const fn new() -> Self {
        Self {
            _not_sendable_by_construction: PhantomData,
        }
    }

    /// Erases the pin's const-generic type while preserving ownership.
    pub fn degrade(self) -> AnyPin {
        AnyPin {
            port: PORT,
            number: N,
        }
    }
}

/// Type-erased, uniquely-owned GPIO pin.
pub struct AnyPin {
    port: u8,
    number: u8,
}

impl AnyPin {
    /// Creates a pin token without consuming [`Pins`].
    ///
    /// # Safety
    ///
    /// The caller must guarantee that no other token for this physical pin
    /// exists for the returned value's lifetime.
    pub unsafe fn steal(port: u8, number: u8) -> Self {
        assert!(port <= 1);
        assert!(number < if port == 0 { 16 } else { 24 });
        Self { port, number }
    }
}

#[doc(hidden)]
pub trait GpioPin {
    fn port(&self) -> u8;
    fn number(&self) -> u8;
}

/// Pin identity used by board descriptions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PinId {
    pub port: u8,
    pub number: u8,
}

impl PinId {
    pub const fn pa(number: u8) -> Self {
        assert!(number < 16);
        Self { port: 0, number }
    }

    pub const fn pb(number: u8) -> Self {
        assert!(number < 24);
        Self { port: 1, number }
    }
}

impl<const PORT: u8, const N: u8> GpioPin for Pin<PORT, N> {
    #[inline]
    fn port(&self) -> u8 {
        PORT
    }

    #[inline]
    fn number(&self) -> u8 {
        N
    }
}

impl GpioPin for AnyPin {
    #[inline]
    fn port(&self) -> u8 {
        self.port
    }

    #[inline]
    fn number(&self) -> u8 {
        self.number
    }
}

fn block(pin: &impl GpioPin) -> &'static pac::gpioa::RegisterBlock {
    match pin.port() {
        0 => unsafe { &*pac::GPIOA::PTR },
        1 => unsafe { &*pac::GPIOB::PTR },
        _ => unreachable!(),
    }
}

/// A pin that can be reconfigured between input and output modes.
pub struct Flex<P: GpioPin> {
    pin: P,
}

impl<P: GpioPin> Flex<P> {
    pub fn new(pin: P) -> Self {
        Self { pin }
    }

    pub fn set_as_input(&mut self, pull: Pull) {
        let registers = block(&self.pin);
        let mask = 1u32 << self.pin.number();
        critical_section::with(|_| unsafe {
            registers.dir().modify(|r, w| w.bits(r.bits() & !mask));
            match pull {
                Pull::None => {
                    registers.pu().modify(|r, w| w.bits(r.bits() & !mask));
                    registers.pd_drv().modify(|r, w| w.bits(r.bits() & !mask));
                }
                Pull::Up => {
                    registers.pu().modify(|r, w| w.bits(r.bits() | mask));
                    registers.pd_drv().modify(|r, w| w.bits(r.bits() & !mask));
                }
                Pull::Down => {
                    registers.pu().modify(|r, w| w.bits(r.bits() & !mask));
                    registers.pd_drv().modify(|r, w| w.bits(r.bits() | mask));
                }
            }
        });
    }

    pub fn set_as_output(&mut self, drive: Drive) {
        let registers = block(&self.pin);
        let mask = 1u32 << self.pin.number();
        critical_section::with(|_| unsafe {
            registers.pd_drv().modify(|r, w| {
                let bits = match drive {
                    Drive::MilliAmps5 => r.bits() & !mask,
                    Drive::MilliAmps20 => r.bits() | mask,
                };
                w.bits(bits)
            });
            registers.dir().modify(|r, w| w.bits(r.bits() | mask));
        });
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        block(&self.pin).pin().read().bits() & (1 << self.pin.number()) != 0
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        block(&self.pin).out().read().bits() & (1 << self.pin.number()) != 0
    }

    pub fn set_level(&mut self, level: Level) {
        let registers = block(&self.pin);
        let mask = 1u32 << self.pin.number();
        critical_section::with(|_| unsafe {
            match level {
                Level::Low => registers.clr().write(|w| w.bits(mask)),
                Level::High => registers.out().modify(|r, w| w.bits(r.bits() | mask)),
            }
        });
    }
}

/// Digital input pin.
pub struct Input<P: GpioPin>(Flex<P>);

impl<P: GpioPin> Input<P> {
    pub fn new(pin: P, pull: Pull) -> Self {
        let mut flex = Flex::new(pin);
        flex.set_as_input(pull);
        Self(flex)
    }

    pub fn degrade(self) -> Input<AnyPin>
    where
        P: Into<AnyPin>,
    {
        Input(Flex::new(self.0.pin.into()))
    }
}

impl<P: GpioPin> ErrorType for Input<P> {
    type Error = Infallible;
}

impl<P: GpioPin> InputPin for Input<P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.0.is_high())
    }
}

/// Push-pull digital output pin.
pub struct Output<P: GpioPin>(Flex<P>);

impl<P: GpioPin> Output<P> {
    pub fn new(pin: P, initial: Level, drive: Drive) -> Self {
        let mut flex = Flex::new(pin);
        flex.set_level(initial);
        flex.set_as_output(drive);
        Self(flex)
    }

    pub fn degrade(self) -> Output<AnyPin>
    where
        P: Into<AnyPin>,
    {
        Output(Flex::new(self.0.pin.into()))
    }
}

impl<P: GpioPin> ErrorType for Output<P> {
    type Error = Infallible;
}

impl<P: GpioPin> OutputPin for Output<P> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_level(Level::High);
        Ok(())
    }
}

impl<P: GpioPin> StatefulOutputPin for Output<P> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.0.is_set_high())
    }
}

macro_rules! pins {
    ($($field:ident: ($port:literal, $number:literal)),+ $(,)?) => {
        /// All GPIOs, obtained by consuming both PAC port tokens.
        pub struct Pins {
            $(pub $field: Pin<$port, $number>,)+
        }

        impl Pins {
            pub fn new(_gpioa: pac::GPIOA, _gpiob: pac::GPIOB) -> Self {
                Self {
                    $($field: Pin::new(),)+
                }
            }
        }

        $(impl From<Pin<$port, $number>> for AnyPin {
            fn from(pin: Pin<$port, $number>) -> Self {
                pin.degrade()
            }
        })+
    };
}

pins! {
    pa0: (0, 0), pa1: (0, 1), pa2: (0, 2), pa3: (0, 3),
    pa4: (0, 4), pa5: (0, 5), pa6: (0, 6), pa7: (0, 7),
    pa8: (0, 8), pa9: (0, 9), pa10: (0, 10), pa11: (0, 11),
    pa12: (0, 12), pa13: (0, 13), pa14: (0, 14), pa15: (0, 15),
    pb0: (1, 0), pb1: (1, 1), pb2: (1, 2), pb3: (1, 3),
    pb4: (1, 4), pb5: (1, 5), pb6: (1, 6), pb7: (1, 7),
    pb8: (1, 8), pb9: (1, 9), pb10: (1, 10), pb11: (1, 11),
    pb12: (1, 12), pb13: (1, 13), pb14: (1, 14), pb15: (1, 15),
    pb16: (1, 16), pb17: (1, 17), pb18: (1, 18), pb19: (1, 19),
    pb20: (1, 20), pb21: (1, 21), pb22: (1, 22), pb23: (1, 23),
}

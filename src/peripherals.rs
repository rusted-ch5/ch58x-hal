//! Uniquely owned CH58x peripheral tokens.

use crate::pac;

/// Token for the emulated DataFlash controller.
#[allow(non_camel_case_types)]
pub struct DATAFLASH {
    _private: (),
}

impl DATAFLASH {
    /// Creates a DataFlash ownership token without checking the HAL singleton.
    ///
    /// This is intended for runtime crates that own the complete peripheral
    /// inventory and enforce their own singleton gate.
    ///
    /// # Safety
    ///
    /// The caller must ensure that no other `DATAFLASH` token exists.
    pub unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

/// Application-owned peripherals returned by [`crate::take`].
#[allow(non_snake_case)]
pub struct Peripherals {
    pub TMR0: pac::TMR0,
    pub TMR1: pac::TMR1,
    pub TMR2: pac::TMR2,
    pub TMR3: pac::TMR3,
    pub UART0: pac::UART0,
    pub UART1: pac::UART1,
    pub UART2: pac::UART2,
    pub UART3: pac::UART3,
    pub SPI0: pac::SPI0,
    pub SPI1: pac::SPI1,
    pub I2C: pac::I2C,
    pub PWMX: pac::PWMX,
    pub USB: pac::USB,
    pub USB2: pac::USB2,
    pub ADC: pac::ADC,
    pub GPIOA: pac::GPIOA,
    pub GPIOB: pac::GPIOB,
    pub RTC: pac::RTC,
    pub DATAFLASH: DATAFLASH,
}

pub(crate) fn from_pac(peripherals: pac::Peripherals) -> Peripherals {
    Peripherals {
        TMR0: peripherals.TMR0,
        TMR1: peripherals.TMR1,
        TMR2: peripherals.TMR2,
        TMR3: peripherals.TMR3,
        UART0: peripherals.UART0,
        UART1: peripherals.UART1,
        UART2: peripherals.UART2,
        UART3: peripherals.UART3,
        SPI0: peripherals.SPI0,
        SPI1: peripherals.SPI1,
        I2C: peripherals.I2C,
        PWMX: peripherals.PWMX,
        USB: peripherals.USB,
        USB2: peripherals.USB2,
        ADC: peripherals.ADC,
        GPIOA: peripherals.GPIOA,
        GPIOB: peripherals.GPIOB,
        RTC: peripherals.RTC,
        DATAFLASH: DATAFLASH { _private: () },
    }
}

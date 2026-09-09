//! Register-level CH58x real-time counter.
//!
//! The counter is read high-low-high to avoid a torn value at a two-second
//! rollover.

use crate::pac;

pub const TICKS_PER_SECOND: u32 = 32_768;

pub struct Rtc {
    _rtc: pac::RTC,
}

impl Rtc {
    pub fn new(rtc: pac::RTC) -> Self {
        Self { _rtc: rtc }
    }

    /// Returns the wrapping 32 kHz tick count within the current day domain.
    pub fn counter_ticks(&self) -> u32 {
        let registers = unsafe { &*pac::RTC::PTR };
        loop {
            let high_before = registers.cnt_2s().read().bits();
            let low = registers.cnt_32k().read().bits();
            let high_after = registers.cnt_2s().read().bits();
            if high_before == high_after {
                return (u32::from(high_before) << 16) | u32::from(low);
            }
        }
    }

    pub fn day(&self) -> u16 {
        let registers = unsafe { &*pac::RTC::PTR };
        registers.cnt_day().read().cnt_day().bits()
    }

    pub fn elapsed_ticks(start: u32, end: u32) -> u32 {
        end.wrapping_sub(start)
    }
}

#![no_std]

#[cfg(all(feature = "ch582", feature = "ch585"))]
compile_error!("ch58x-hal chip features ch582 and ch585 are mutually exclusive");
#[cfg(not(any(feature = "ch582", feature = "ch585")))]
compile_error!("ch58x-hal requires exactly one chip feature: ch582 or ch585");

use portable_atomic::{AtomicBool, Ordering};

pub use ch58x::ch58x as pac;

mod critical_section_impl;
pub mod peripherals;

pub use peripherals::Peripherals;

static PERIPHERALS_TAKEN: AtomicBool = AtomicBool::new(false);

/// Takes all application-owned peripheral tokens exactly once.
pub fn take() -> Option<Peripherals> {
    if PERIPHERALS_TAKEN
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return None;
    }

    // SAFETY: the atomic singleton gate above admits exactly one caller.
    Some(peripherals::from_pac(unsafe { pac::Peripherals::steal() }))
}

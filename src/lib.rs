#![no_std]

#[cfg(all(feature = "ch582", feature = "ch585"))]
compile_error!("ch58x-hal chip features ch582 and ch585 are mutually exclusive");
#[cfg(not(any(feature = "ch582", feature = "ch585")))]
compile_error!("ch58x-hal requires exactly one chip feature: ch582 or ch585");

#[cfg(feature = "ch582")]
use core::ptr;
use portable_atomic::{AtomicBool, Ordering};

pub use ch58x::ch58x as pac;

pub mod adc;
mod critical_section_impl;
#[cfg(feature = "ch582")]
pub mod dataflash;
#[cfg(feature = "ch582")]
mod dataflash_protocol;
pub mod gpio;
pub mod peripherals;
pub mod rtc;
pub mod sysctl;
#[cfg(feature = "uart")]
pub mod uart;

pub use peripherals::Peripherals;

static PERIPHERALS_TAKEN: AtomicBool = AtomicBool::new(false);

/// Performs one protected CH58x system-register transaction.
#[cfg(feature = "ch582")]
pub(crate) fn with_safe_access<R>(f: impl FnOnce() -> R) -> R {
    const SAFE_ACCESS: *mut u8 = 0x4000_1040 as *mut u8;

    critical_section::with(|_| unsafe {
        ptr::write_volatile(SAFE_ACCESS, 0x57);
        ptr::write_volatile(SAFE_ACCESS, 0xa8);
        qingke::riscv::asm::nop();
        qingke::riscv::asm::nop();
        let result = f();
        ptr::write_volatile(SAFE_ACCESS, 0);
        result
    })
}

fn claim_peripherals() -> bool {
    PERIPHERALS_TAKEN
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
}

/// Configures the system clock and returns all application-owned peripherals.
pub fn init(config: sysctl::Config) -> Peripherals {
    assert!(claim_peripherals(), "ch58x-hal initialized twice");
    config.freeze();

    // SAFETY: the atomic singleton gate above admits exactly one caller.
    peripherals::from_pac(unsafe { pac::Peripherals::steal() })
}

/// Takes all application-owned peripheral tokens exactly once.
pub fn take() -> Option<Peripherals> {
    if !claim_peripherals() {
        return None;
    }

    // SAFETY: the atomic singleton gate above admits exactly one caller.
    Some(peripherals::from_pac(unsafe { pac::Peripherals::steal() }))
}

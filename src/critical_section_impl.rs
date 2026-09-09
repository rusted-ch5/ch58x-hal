//! CH58x single-hart critical-section implementation.
//!
//! GINTENR writes are followed by two instructions so the CSR update has
//! cleared the QingKe pipeline before execution continues.

use core::sync::atomic::{Ordering, compiler_fence};
use critical_section::{Impl, RawRestoreState, set_impl};

const GLOBAL_INTERRUPT_ENABLE: usize = 0x08;
const GLOBAL_INTERRUPT_STATE_MASK: usize = 0x88;

struct Ch58xCriticalSection;
set_impl!(Ch58xCriticalSection);

#[inline(always)]
fn disable_and_save() -> bool {
    let previous: usize;
    unsafe {
        core::arch::asm!(
            "csrrc {previous}, 0x800, {mask}",
            "nop",
            "nop",
            previous = out(reg) previous,
            mask = in(reg) GLOBAL_INTERRUPT_STATE_MASK,
            options(nostack),
        );
    }
    compiler_fence(Ordering::SeqCst);
    previous & GLOBAL_INTERRUPT_ENABLE != 0
}

#[inline(always)]
unsafe fn enable_interrupts() {
    unsafe {
        core::arch::asm!(
            "csrs 0x800, {mask}",
            "nop",
            "nop",
            mask = in(reg) GLOBAL_INTERRUPT_STATE_MASK,
            options(nostack),
        );
    }
}

unsafe impl Impl for Ch58xCriticalSection {
    #[inline(always)]
    unsafe fn acquire() -> RawRestoreState {
        disable_and_save()
    }

    #[inline(always)]
    unsafe fn release(was_enabled: RawRestoreState) {
        compiler_fence(Ordering::SeqCst);
        if was_enabled {
            unsafe { enable_interrupts() };
        }
    }
}

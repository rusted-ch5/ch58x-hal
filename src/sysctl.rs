//! Clock setup for CH58x.

use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "ch582")]
use crate::pac;
#[cfg(feature = "ch582")]
use crate::with_safe_access;

static HCLK_HZ: AtomicU32 = AtomicU32::new(6_400_000);

#[cfg(feature = "ch585")]
#[inline(always)]
fn with_ch585_safe_access<R>(f: impl FnOnce() -> R) -> R {
    use core::ptr::{read_volatile, write_volatile};

    const SAFE_MODE_CTRL: *mut u8 = 0x4000_1010 as *mut u8;
    const SAFE_ACCESS: *mut u8 = 0x4000_1040 as *mut u8;
    const SAFE_AUTO_EN: u8 = 0x01;

    critical_section::with(|_| unsafe {
        // CH585's public SetSysClock() disables the approximately 16-cycle
        // automatic safe-access timeout before opening the protected window.
        // The oscillator settle loop and clock/Flash handoff cannot be split
        // across ordinary short CH58x safe-access transactions.
        write_volatile(
            SAFE_MODE_CTRL,
            read_volatile(SAFE_MODE_CTRL) & !SAFE_AUTO_EN,
        );
        core::arch::asm!("fence.i", options(nostack));
        write_volatile(SAFE_ACCESS, 0x57);
        write_volatile(SAFE_ACCESS, 0xa8);
        core::arch::asm!("fence.i", options(nostack));

        let result = f();

        write_volatile(SAFE_MODE_CTRL, read_volatile(SAFE_MODE_CTRL) | SAFE_AUTO_EN);
        write_volatile(SAFE_ACCESS, 0);
        core::arch::asm!("fence.i", options(nostack));
        result
    })
}

/// Returns the configured high-speed system clock.
pub fn hclk_hz() -> u32 {
    HCLK_HZ.load(Ordering::Relaxed)
}

/// System-clock configuration.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Core clock in hertz.
    pub hclk_hz: u32,
    /// Board has the external 32 MHz crystal required by the 480 MHz PLL.
    pub enable_hse: bool,
    /// Use an external 32.768 kHz crystal. When false, the internal 32 kHz RC
    /// remains selected and the controller must periodically calibrate it.
    pub enable_lse: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            #[cfg(feature = "ch582")]
            hclk_hz: 60_000_000,
            #[cfg(feature = "ch585")]
            hclk_hz: 62_400_000,
            enable_hse: true,
            enable_lse: false,
        }
    }
}

impl Config {
    #[cfg(feature = "ch582")]
    pub(crate) fn freeze(self) {
        // The first implementation deliberately supports one well-tested PLL
        // configuration. Other dividers must not silently desynchronise
        // timing-sensitive peripheral drivers.
        assert_eq!(self.hclk_hz, 60_000_000);
        assert!(
            self.enable_hse,
            "60 MHz PLL clock requires an external 32 MHz crystal"
        );
        let sys = unsafe { &*pac::SYS::PTR };

        // A bootloader or previously running application may leave the WDT
        // enabled with a nearly expired counter. Reload it before oscillator
        // startup delays; writing the count does not enable the watchdog.
        sys.wdog_count().write(|w| unsafe { w.bits(0) });

        if self.enable_lse {
            with_safe_access(|| {
                sys.ck32k_config()
                    .modify(|_, w| w.clk_xt32k_pon().set_bit());
            });
            qingke::riscv::asm::delay(640_000);
            sys.wdog_count().write(|w| unsafe { w.bits(0) });
            with_safe_access(|| {
                sys.ck32k_config()
                    .modify(|_, w| w.clk_osc32k_xt().set_bit());
            });
        } else {
            with_safe_access(|| {
                sys.ck32k_config()
                    .modify(|_, w| w.clk_osc32k_xt().clear_bit().clk_int32k_pon().set_bit());
            });
        }

        // The PLL input is the external 32 MHz crystal. Power it explicitly;
        // relying on its reset value makes startup depend on bootloader state.
        if sys.hfck_pwr_ctrl().read().clk_xt32m_pon().bit_is_clear() {
            with_safe_access(|| {
                sys.hfck_pwr_ctrl()
                    .modify(|_, w| w.clk_xt32m_pon().set_bit());
            });
            // WCH's public clock routine waits 2400 reset-clock cycles.
            qingke::riscv::asm::delay(2_400);
        }
        // Enable the 480 MHz PLL, wait for it to settle, then divide by eight.
        // CLK_SYS_MOD=01 selects the PLL. Protected writes are kept separate so
        // each operation fits inside the hardware's safe-access window.
        with_safe_access(|| {
            sys.pll_config()
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 5)) });
        });
        if sys.hfck_pwr_ctrl().read().clk_pll_pon().bit_is_clear() {
            with_safe_access(|| {
                sys.hfck_pwr_ctrl().modify(|_, w| w.clk_pll_pon().set_bit());
            });
            // WCH's public clock routine waits 4000 reset-clock cycles.
            qingke::riscv::asm::delay(4_000);
        }
        with_safe_access(|| unsafe {
            sys.clk_sys_cfg()
                .write(|w| w.clk_sys_mod().bits(0b01).clk_pll_div().bits(8));
            qingke::riscv::asm::nop();
            qingke::riscv::asm::nop();
            qingke::riscv::asm::nop();
            qingke::riscv::asm::nop();
        });
        with_safe_access(|| unsafe {
            sys.flash_cfg().write(|w| w.bits(0x52));
        });
        with_safe_access(|| {
            sys.pll_config()
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 7)) });
        });
        HCLK_HZ.store(self.hclk_hz, Ordering::Relaxed);
    }

    #[cfg(feature = "ch585")]
    #[inline(never)]
    #[unsafe(link_section = ".highcode.ch585_clock")]
    pub(crate) fn freeze(self) {
        use core::ptr::{read_volatile, write_volatile};

        const CLK_SYS_CFG: *mut u16 = 0x4000_1008 as *mut u16;
        const HFCK_PWR_CTRL: *mut u8 = 0x4000_100a as *mut u8;
        const SAFE_MODE_CTRL: *mut u32 = 0x4000_1010 as *mut u32;
        const CK32K_CONFIG: *mut u8 = 0x4000_102f as *mut u8;
        const WATCHDOG_COUNT: *mut u8 = 0x4000_1043 as *mut u8;
        const MISC_CTRL: *mut u32 = 0x4000_1048 as *mut u32;
        const PLL_CONFIG: *mut u8 = 0x4000_104b as *mut u8;
        const XT32M_TUNE: *mut u8 = 0x4000_104e as *mut u8;
        const FLASH_SCK: *mut u8 = 0x4000_1805 as *mut u8;
        const FLASH_CFG: *mut u8 = 0x4000_1807 as *mut u8;

        const CLK_RC16M_PON: u8 = 0x02;
        const CLK_XT32M_PON: u8 = 0x04;
        const CLK_PLL_PON: u8 = 0x10;
        const CLK_XT32K_PON: u8 = 0x01;
        const CLK_INT32K_PON: u8 = 0x02;
        const CLK_OSC32K_XT: u8 = 0x04;
        // Internal 16 MHz -> 312 MHz XROM PLL path -> divide by 5.
        const HSI_PLL_62M4: u16 = 0x0145;
        // External 32 MHz -> 624 MHz PLL -> divide by 5 = 62.4 MHz.
        const HSE_PLL_62M4: u16 = 0x0345;

        assert_eq!(self.hclk_hz, 62_400_000);
        assert!(
            self.enable_hse,
            "62.4 MHz CH585 PLL clock requires an external 32 MHz crystal"
        );
        unsafe { write_volatile(WATCHDOG_COUNT, 0) };

        // Reproduce public highcode_init() before selecting the final HSE
        // clock. The upstream QingKe runtime does not provide this CH585-only
        // SoC initialization hook, so it must run here from copied RAM code.
        with_ch585_safe_access(|| unsafe {
            write_volatile(SAFE_MODE_CTRL, read_volatile(SAFE_MODE_CTRL) | 0x10);
            write_volatile(MISC_CTRL, read_volatile(MISC_CTRL) | 5 | (3 << 25));
            write_volatile(PLL_CONFIG, read_volatile(PLL_CONFIG) & !(1 << 5));
            write_volatile(
                HFCK_PWR_CTRL,
                read_volatile(HFCK_PWR_CTRL) | CLK_RC16M_PON | CLK_PLL_PON,
            );
            write_volatile(CLK_SYS_CFG, HSI_PLL_62M4);
            write_volatile(FLASH_SCK, read_volatile(FLASH_SCK) & !(1 << 4));
            write_volatile(FLASH_CFG, 0x02);
            write_volatile(XT32M_TUNE, (read_volatile(XT32M_TUNE) & !0x03) | 0x01);
            write_volatile(CK32K_CONFIG, read_volatile(CK32K_CONFIG) | CLK_INT32K_PON);
        });
        if self.enable_lse {
            with_ch585_safe_access(|| unsafe {
                write_volatile(CK32K_CONFIG, read_volatile(CK32K_CONFIG) | CLK_XT32K_PON);
            });
            qingke::riscv::asm::delay(640_000);
            unsafe { write_volatile(WATCHDOG_COUNT, 0) };
            with_ch585_safe_access(|| unsafe {
                write_volatile(CK32K_CONFIG, read_volatile(CK32K_CONFIG) | CLK_OSC32K_XT);
            });
        } else {
            with_ch585_safe_access(|| unsafe {
                let value = (read_volatile(CK32K_CONFIG) & !CLK_OSC32K_XT) | CLK_INT32K_PON;
                write_volatile(CK32K_CONFIG, value);
            });
        }
        // Public SetSysClock() performs the HSE startup and final clock/Flash
        // handoff in one non-expiring safe-access window from RAM.
        with_ch585_safe_access(|| unsafe {
            write_volatile(SAFE_MODE_CTRL, read_volatile(SAFE_MODE_CTRL) | 0x10);
            if read_volatile(HFCK_PWR_CTRL) & CLK_XT32M_PON == 0 {
                let tune = read_volatile(XT32M_TUNE);
                write_volatile(XT32M_TUNE, tune | 0x03);
                write_volatile(HFCK_PWR_CTRL, read_volatile(HFCK_PWR_CTRL) | CLK_XT32M_PON);
                let previous = read_volatile(CLK_SYS_CFG);
                write_volatile(CLK_SYS_CFG, previous | 0x00c0);
                for _ in 0..9 {
                    qingke::riscv::asm::nop();
                }
                write_volatile(CLK_SYS_CFG, previous);
                write_volatile(XT32M_TUNE, tune);
            }
            write_volatile(HFCK_PWR_CTRL, read_volatile(HFCK_PWR_CTRL) | CLK_PLL_PON);
            write_volatile(FLASH_SCK, read_volatile(FLASH_SCK) & !(1 << 4));
            write_volatile(FLASH_CFG, 0x01);
            // The transient direct-32K selection closes the PLL gate before
            // committing the requested source, matching SetSysClock().
            write_volatile(CLK_SYS_CFG, HSE_PLL_62M4 | 0x00c0);
            write_volatile(CLK_SYS_CFG, HSE_PLL_62M4);
        });
        HCLK_HZ.store(self.hclk_hz, Ordering::Relaxed);
    }
}

//! Register-level CH582 DataFlash driver.
//!
//! The public view is deliberately limited to the final 8 KiB of the 32-KiB
//! DataFlash region. Program flash, bootloader flash and chip configuration are
//! not addressable through this type.

use core::ptr::{read_volatile, write_volatile};

use embedded_storage_async::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::dataflash_protocol::status_ready;

const SAFE_ACCESS: *mut u8 = 0x4000_1040 as *mut u8;
const GLOBAL_CONFIG: *mut u8 = 0x4000_1044 as *mut u8;
const FLASH_DATA: *mut u8 = 0x4000_1804 as *mut u8;
const FLASH_CTRL: *mut u8 = 0x4000_1806 as *mut u8;

const DATA_FLASH_BASE: u32 = 0x0007_0000;
const DATA_FLASH_ALIAS: u32 = 0x0008_0000;
const EXPOSED_OFFSET: u32 = 0x0000_6000;
const CAPACITY: usize = 8 * 1024;
const ERASE_SIZE: usize = 256;
const PAGE_SIZE: usize = 256;
const MAX_STATUS_POLLS: usize = 0x80_000;

const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_READ_STATUS: u8 = 0x05;
const CMD_DATA_READ: u8 = 0x0b;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_ERASE_256: u8 = 0x81;
const CMD_CONTROLLER_RESET: u8 = 0xff;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    OutOfBounds,
    NotAligned,
    Timeout,
    Program,
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Timeout | Self::Program => NorFlashErrorKind::Other,
        }
    }
}

// CH58x executes these transactions from SRAM. Once a command byte is written
// to the internal Flash controller, fetching the following instruction from
// code Flash can deadlock the hart. This is the same placement contract as the
// controller's `.highcode` Flash primitives; the implementation below is
// independent, register-level Rust.

#[inline(always)]
fn ram_wait_interface() -> bool {
    for _ in 0..MAX_STATUS_POLLS {
        if unsafe { read_volatile(FLASH_CTRL) } & 0x80 == 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

#[inline(always)]
fn ram_begin(command: u8) {
    unsafe {
        write_volatile(FLASH_CTRL, 0);
        write_volatile(FLASH_CTRL, 5);
        write_volatile(FLASH_DATA, command);
    }
}

#[inline(always)]
fn ram_output(byte: u8) -> bool {
    if !ram_wait_interface() {
        return false;
    }
    unsafe { write_volatile(FLASH_DATA, byte) };
    true
}

// This serializes only transaction setup; byte output remains fully inlined
// because the controller requires a prompt write after its busy flag clears.
#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_address(command: u8, address: u32, read_dummy: bool) -> bool {
    ram_begin(command);
    if !ram_output((address >> 16) as u8)
        || !ram_output((address >> 8) as u8)
        || !ram_output(address as u8)
    {
        return false;
    }
    !read_dummy || (ram_output(0) && ram_output(0))
}

#[inline(always)]
fn ram_end() -> bool {
    let ready = ram_wait_interface();
    unsafe { write_volatile(FLASH_CTRL, 0) };
    ready
}

#[inline(always)]
fn ram_abort() -> Error {
    unsafe { write_volatile(FLASH_CTRL, 0) };
    Error::Timeout
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_configure(write: bool) -> u8 {
    let original = unsafe { read_volatile(GLOBAL_CONFIG) };
    unsafe {
        write_volatile(SAFE_ACCESS, 0x57);
        write_volatile(SAFE_ACCESS, 0xa8);
        core::arch::asm!("nop", "nop", options(nomem, nostack));
        write_volatile(GLOBAL_CONFIG, original | if write { 0xe0 } else { 0x20 });
        write_volatile(SAFE_ACCESS, 0);
    }
    original
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_restore(original: u8) {
    unsafe {
        write_volatile(SAFE_ACCESS, 0x57);
        write_volatile(SAFE_ACCESS, 0xa8);
        core::arch::asm!("nop", "nop", options(nomem, nostack));
        write_volatile(GLOBAL_CONFIG, original);
        write_volatile(SAFE_ACCESS, 0);
    }
}

#[inline(always)]
fn ram_fail(original: u8) -> Error {
    let error = ram_abort();
    ram_restore(original);
    error
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_read_bytes(address: u32, output: *mut u8, length: usize) -> Result<(), Error> {
    let original = ram_configure(false);
    if ram_prepare().is_err() {
        ram_restore(original);
        return Err(Error::Timeout);
    }
    if !ram_address(CMD_DATA_READ, address, true) {
        return Err(ram_fail(original));
    }
    for index in 0..length {
        if !ram_wait_interface() {
            return Err(ram_fail(original));
        }
        unsafe { output.add(index).write(read_volatile(FLASH_DATA)) };
    }
    if !ram_end() {
        ram_restore(original);
        return Err(Error::Timeout);
    }
    ram_restore(original);
    Ok(())
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_prepare() -> Result<(), Error> {
    // The CH58x ISP command dispatcher primes the Flash interface this way
    // before every EEPROM/ROM-info operation. R8_FLASH_CTRL=4 selects the ROM
    // command path; 0xff then resets its serial state machine.
    unsafe { write_volatile(FLASH_CTRL, 4) };
    ram_begin(CMD_CONTROLLER_RESET);
    if !ram_end() {
        return Err(Error::Timeout);
    }
    Ok(())
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_wait_ready_active() -> bool {
    for _ in 0..MAX_STATUS_POLLS {
        ram_begin(CMD_READ_STATUS);
        if !ram_wait_interface() {
            return false;
        }
        let first = unsafe { read_volatile(FLASH_DATA) };
        if !ram_wait_interface() {
            return false;
        }
        let second = unsafe { read_volatile(FLASH_DATA) };
        if !ram_end() {
            return false;
        }
        if status_ready([first, second]) {
            return true;
        }
    }
    false
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_erase(address: u32) -> Result<(), Error> {
    let original = ram_configure(true);
    if ram_prepare().is_err() {
        ram_restore(original);
        return Err(Error::Timeout);
    }
    ram_begin(CMD_WRITE_ENABLE);
    if !ram_end() {
        return Err(ram_fail(original));
    }
    if !ram_address(CMD_ERASE_256, address, false) || !ram_end() {
        return Err(ram_fail(original));
    }
    if !ram_wait_ready_active() {
        return Err(ram_fail(original));
    }
    ram_restore(original);
    Ok(())
}

#[unsafe(link_section = ".highcode.dataflash")]
#[inline(never)]
fn ram_page_program(address: u32, data: *const u8, length: usize) -> Result<(), Error> {
    let original = ram_configure(true);
    if ram_prepare().is_err() {
        ram_restore(original);
        return Err(Error::Timeout);
    }
    ram_begin(CMD_WRITE_ENABLE);
    if !ram_end() {
        return Err(ram_fail(original));
    }
    if !ram_address(CMD_PAGE_PROGRAM, address, false) {
        return Err(ram_fail(original));
    }
    for index in 0..length {
        if !ram_output(unsafe { data.add(index).read() }) {
            return Err(ram_fail(original));
        }
    }
    if !ram_end() {
        return Err(ram_fail(original));
    }
    if !ram_wait_ready_active() {
        return Err(ram_fail(original));
    }
    ram_restore(original);
    Ok(())
}

/// Exclusive access token for the reserved DataFlash window.
pub struct DataFlash {
    _token: crate::peripherals::DATAFLASH,
}

impl DataFlash {
    /// Creates the unique DataFlash driver from the token returned by
    /// [`crate::init`].
    pub fn new(token: crate::peripherals::DATAFLASH) -> Self {
        Self { _token: token }
    }

    fn physical(offset: usize) -> Result<u32, Error> {
        if offset >= CAPACITY {
            return Err(Error::OutOfBounds);
        }
        Ok(DATA_FLASH_ALIAS | (DATA_FLASH_BASE + EXPOSED_OFFSET + offset as u32))
    }

    fn with_access<T>(
        _write: bool,
        operation: impl FnOnce() -> Result<T, Error>,
    ) -> Result<T, Error> {
        critical_section::with(|_| operation())
    }
}

impl ErrorType for DataFlash {
    type Error = Error;
}

impl ReadNorFlash for DataFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, output: &mut [u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        let end = start.checked_add(output.len()).ok_or(Error::OutOfBounds)?;
        if end > CAPACITY {
            return Err(Error::OutOfBounds);
        }
        if output.is_empty() {
            return Ok(());
        }
        Self::with_access(false, || {
            let address = Self::physical(start)?;
            ram_read_bytes(address, output.as_mut_ptr(), output.len())
        })
    }

    fn capacity(&self) -> usize {
        CAPACITY
    }
}

impl NorFlash for DataFlash {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let from = from as usize;
        let to = to as usize;
        if from & (ERASE_SIZE - 1) != 0 || to & (ERASE_SIZE - 1) != 0 {
            return Err(Error::NotAligned);
        }
        if from >= to || to > CAPACITY {
            return Err(Error::OutOfBounds);
        }
        Self::with_access(true, || {
            for offset in (from..to).step_by(ERASE_SIZE) {
                ram_erase(Self::physical(offset)?)?;
            }
            Ok(())
        })
    }

    async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        let end = start.checked_add(data.len()).ok_or(Error::OutOfBounds)?;
        if end > CAPACITY {
            return Err(Error::OutOfBounds);
        }
        Self::with_access(true, || {
            let mut scratch = [0u8; 32];
            let mut cursor = 0;
            while cursor < data.len() {
                let address = start + cursor;
                let page_remaining = PAGE_SIZE - (address % PAGE_SIZE);
                let count = page_remaining.min(data.len() - cursor).min(scratch.len());
                scratch[..count].copy_from_slice(&data[cursor..cursor + count]);
                ram_page_program(Self::physical(address)?, scratch.as_ptr(), count)?;
                cursor += count;
            }
            Ok(())
        })
    }
}

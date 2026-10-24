//! Three-point RF capacitance-tuning table generation.
//!
//! Some integrated radios measure oscillator coarse (`co`) and gain (`ga`)
//! values at three frequency anchors. This pure algorithm builds packed
//! four-bit interpolation corrections; a chip backend owns the measurements
//! and writes the result to its hardware table.

pub const TABLE_WORDS: usize = 13;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Measurement {
    pub co: u8,
    pub ga: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    DegenerateCoarseRange,
}

pub fn build_table(
    mhz_2401: Measurement,
    mhz_2440: Measurement,
    mhz_2480: Measurement,
) -> Result<[u32; TABLE_WORDS], Error> {
    let low_co_delta = i32::from(mhz_2401.co) - i32::from(mhz_2440.co);
    let high_co_delta = i32::from(mhz_2440.co) - i32::from(mhz_2480.co);
    if low_co_delta == 0 || high_co_delta == 0 {
        return Err(Error::DegenerateCoarseRange);
    }

    let low_ga_delta = i32::from(mhz_2401.ga) - i32::from(mhz_2440.ga);
    let high_ga_delta = i32::from(mhz_2440.ga) - i32::from(mhz_2480.ga);
    let mut table = [0u32; TABLE_WORDS];

    for slot in 0..40 {
        let multiplier = 39 - slot as i32;
        put_nibble(&mut table, slot, low_co_delta * multiplier / 39);
    }

    for step in 1..=42 {
        put_nibble(&mut table, 39 + step as usize, high_co_delta * step / 40);
    }

    for step in (1..=10).rev() {
        put_nibble(
            &mut table,
            82 + (10 - step) as usize,
            (high_ga_delta * step / high_co_delta) | 0x08,
        );
    }
    for step in 0..=10 {
        put_nibble(
            &mut table,
            92 + step as usize,
            low_ga_delta * step / low_co_delta,
        );
    }

    Ok(table)
}

fn put_nibble(table: &mut [u32; TABLE_WORDS], slot: usize, value: i32) {
    debug_assert!(slot < TABLE_WORDS * 8);
    let shift = (slot % 8) * 4;
    table[slot / 8] |= ((value as u32) & 0x0f) << shift;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_zero_coarse_span() {
        let same = Measurement { co: 20, ga: 40 };
        assert_eq!(
            build_table(same, same, Measurement { co: 10, ga: 20 }),
            Err(Error::DegenerateCoarseRange)
        );
    }

    #[test]
    fn interpolation_hits_coarse_endpoints() {
        let table = build_table(
            Measurement { co: 50, ga: 60 },
            Measurement { co: 38, ga: 48 },
            Measurement { co: 28, ga: 38 },
        )
        .unwrap();

        assert_eq!(nibble(&table, 0), 12);
        assert_eq!(nibble(&table, 39), 0);
        assert_eq!(nibble(&table, 40), 0);
        assert_eq!(nibble(&table, 79), 10);
        assert_eq!(nibble(&table, 80), 10);
        assert_eq!(nibble(&table, 81), 10);
    }

    #[test]
    fn packs_all_thirteen_hardware_words() {
        let table = build_table(
            Measurement { co: 55, ga: 62 },
            Measurement { co: 39, ga: 50 },
            Measurement { co: 23, ga: 36 },
        )
        .unwrap();
        assert_ne!(table[0], 0);
        assert_ne!(table[12] & 0x0fff_ffff, 0);
        assert_eq!(table[12] >> 28, 0);
    }

    fn nibble(table: &[u32; TABLE_WORDS], slot: usize) -> u8 {
        ((table[slot / 8] >> ((slot % 8) * 4)) & 0x0f) as u8
    }
}

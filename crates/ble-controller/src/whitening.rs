/// Applies the Bluetooth LE data whitening LFSR in place.
pub fn apply(channel: u8, bytes: &mut [u8]) {
    // This is the bit-reversed storage form of Core positions 0 through 6:
    // position 0 is one and positions 1..=6 contain channel[5..=0].
    let mut lfsr = (channel & 0x3f) | 0x40;
    for byte in bytes {
        let mut value = *byte;
        for bit in 0..8 {
            if lfsr & 1 != 0 {
                lfsr ^= 0x88;
                value ^= 1 << bit;
            }
            lfsr >>= 1;
        }
        *byte = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn applying_whitening_twice_recovers_input() {
        let original = *b"rmk-ch582m";
        let mut bytes = original;
        apply(37, &mut bytes);
        assert_ne!(bytes, original);
        apply(37, &mut bytes);
        assert_eq!(bytes, original);
    }

    #[test]
    fn channel_23_initialization_matches_core_example() {
        let mut zeros = [0u8; 2];
        apply(23, &mut zeros);
        assert_eq!(zeros, [0xaf, 0x42]);
    }
}

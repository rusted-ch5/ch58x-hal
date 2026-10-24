//! CH585 W1C receive status ownership. Not the CH582 clear-by-zero policy.

/// Capture IRQ owns bit 6. The RX consumer owns CRC-error bit 5 until it has
/// classified the packet; clearing both here can promote corrupt RX to valid.
pub(crate) const fn capture_ack(status: u32) -> u32 {
    status & (1 << 6)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_ack_preserves_crc_and_other_sources() {
        for status in 0..1024u32 {
            let remaining = status & !capture_ack(status); // W1C effect
            assert_eq!(remaining & (1 << 5), status & (1 << 5));
            assert_eq!(remaining & (1 << 6), 0);
            assert_eq!(remaining & !(1 << 6), status & !(1 << 6));
        }
    }

    #[test]
    fn old_combined_clear_erased_crc_failure() {
        let capture_with_crc_error = 0x60u32;
        assert_eq!(capture_with_crc_error & !(capture_with_crc_error & 0x60), 0);
        assert_eq!(
            capture_with_crc_error & !capture_ack(capture_with_crc_error),
            0x20
        );
    }
}

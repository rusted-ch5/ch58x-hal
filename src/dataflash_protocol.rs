/// Decode the two bytes clocked by the serial Flash status transaction.
///
/// The first byte is a throw-away transfer; the actual status register is the
/// second byte. Bit 0 is the erase/program busy flag.
#[inline(always)]
pub(crate) const fn status_ready(received: [u8; 2]) -> bool {
    received[1] & 1 == 0
}

#[cfg(test)]
mod tests {
    use super::status_ready;

    #[test]
    fn flash_status_uses_second_received_byte() {
        assert!(status_ready([0x01, 0x00]));
        assert!(!status_ready([0x00, 0x01]));
        assert!(status_ready([0xff, 0x02]));
    }
}

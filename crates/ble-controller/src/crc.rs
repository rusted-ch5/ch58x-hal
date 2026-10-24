/// Bluetooth LE CRC24 shift-register state, in positions 0 through 23.
///
/// PDU octets are consumed in transmitted order and each octet is consumed
/// least-significant bit first. The returned value keeps position 23 in bit 23;
/// that position is the first CRC bit transmitted on air.
pub fn crc24(data: &[u8], init: u32) -> u32 {
    let mut state = init & 0x00ff_ffff;
    for &byte in data {
        for bit in 0..8 {
            let input = (byte >> bit) & 1;
            let feedback = ((state >> 23) as u8) ^ input;
            state = (state << 1) & 0x00ff_ffff;
            if feedback != 0 {
                state ^= 0x0000_065b;
            }
        }
    }
    state
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_pdu_preserves_initial_state() {
        assert_eq!(crc24(&[], 0x55_55_55), 0x55_55_55);
    }

    #[test]
    fn advertising_crc_regression_vector() {
        // ADV_IND header, six-octet AdvA, no AdvData. The expected state was
        // calculated coefficient-by-coefficient from Core Vol 6, Part B,
        // section 3.1.1, not from CH58x vendor code.
        assert_eq!(
            crc24(
                &[0x00, 0x06, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6],
                0x55_55_55
            ),
            0xa4_09_2b
        );
    }
}

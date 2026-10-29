//! Pure CH585 LE 1M DMA layout and timing arithmetic.
use crate::radio_dma_policy::RF_DMA_BUFFER_LEN;

pub(crate) const fn legacy_request_payload_len_is_valid(pdu_type: u8, payload_len: usize) -> bool {
    matches!((pdu_type, payload_len), (0x03, 12) | (0x05, 34))
}

pub(crate) fn encode_transmit_buffer(buffer: &mut [u8; RF_DMA_BUFFER_LEN], packet: &[u8]) {
    buffer.fill(0);
    buffer[..packet.len()].copy_from_slice(packet);
}

pub(crate) const fn tx_1m_countdown(packet_len: usize) -> u32 {
    // CH585 BLE_SetPHYTxMode(LE1M), with CTE disabled:
    // (((payload_len + 4) * 8) + 0xee) * 2. `packet_len` already includes
    // the two-byte Link Layer header, so the vendor length argument is
    // `packet_len + 2` rather than `packet_len`.
    ((((packet_len as u32) + 2) * 8) + 0xee) << 1
}

pub(crate) const fn rx_1m_capture_countdown(max_packet_len: usize) -> u32 {
    // CH585 BLE_SetPHYRxMode(LE1M): `(len + 5) * 16 + 0x3c`.
    (((max_packet_len as u32) + 5) << 4) + 0x3c
}

pub(crate) const fn rx_to_tx_timing(_received_len: u8) -> u32 {
    // CH585 LLE_DevInit installs one fixed LE 1M RX-to-TX timing offset.
    0x76
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn header_is_not_a_complete_request_until_length_is_exact() {
        assert!(legacy_request_payload_len_is_valid(3, 12));
        assert!(legacy_request_payload_len_is_valid(5, 34));
        for length in [0, 1, 12, 33, 35, 63] {
            assert!(!legacy_request_payload_len_is_valid(5, length));
        }
    }
    #[test]
    fn radio_timing_matches_ch585_legacy_register_vectors() {
        assert_eq!(tx_1m_countdown(31), 1004);
        assert_eq!(rx_1m_capture_countdown(37), 0x2dc);
        assert_eq!(rx_1m_capture_countdown(27), 0x23c);
        assert_eq!(rx_to_tx_timing(0), 0x76);
        assert_eq!(rx_to_tx_timing(27), 0x76);
    }
    #[test]
    fn dma_layout_is_contiguous_and_does_not_keep_old_payload() {
        let mut bank = [0xff; RF_DMA_BUFFER_LEN];
        let packet = [0x40, 0x08, 1, 2, 3, 4, 5, 0xc0, 0xaa, 0xbb];
        encode_transmit_buffer(&mut bank, &packet);
        assert_eq!(&bank[..packet.len()], &packet);
        assert!(bank[packet.len()..].iter().all(|&byte| byte == 0));
    }
}

//! Pure length policy for the CH585 plaintext RF DMA banks.

pub const MAX_LL_DATA_PAYLOAD_LEN: usize = 27;
pub const SUPPORTS_DATA_LENGTH_EXTENSION: bool = false;
pub const SUPPORTS_EXTENDED_ADVERTISING: bool = false;
pub const MAX_LEGACY_ADV_PAYLOAD_LEN: usize = 37;
pub const RF_DMA_BUFFER_LEN: usize = 64;
pub const LL_HEADER_LEN: usize = 2;
pub const MAX_LEGACY_ADV_WIRE_LEN: usize = LL_HEADER_LEN + MAX_LEGACY_ADV_PAYLOAD_LEN;
pub const MAX_DATA_WIRE_LEN: usize = LL_HEADER_LEN + MAX_LL_DATA_PAYLOAD_LEN;
pub const CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN: usize = MAX_LL_DATA_PAYLOAD_LEN;

const _: () = {
    assert!(!SUPPORTS_DATA_LENGTH_EXTENSION);
    assert!(!SUPPORTS_EXTENDED_ADVERTISING);
    assert!(MAX_LEGACY_ADV_WIRE_LEN <= RF_DMA_BUFFER_LEN);
    assert!(MAX_DATA_WIRE_LEN <= RF_DMA_BUFFER_LEN);
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReceiveLengthPolicy {
    Accepted { wire_len: usize },
    PhysicalDmaOverflow,
    UnsupportedCapability,
}

pub const fn classify_received_length(
    advertising: bool,
    payload_len: usize,
) -> ReceiveLengthPolicy {
    let wire_len = LL_HEADER_LEN + payload_len;
    if wire_len > RF_DMA_BUFFER_LEN {
        return ReceiveLengthPolicy::PhysicalDmaOverflow;
    }
    let maximum_wire_len = if advertising {
        MAX_LEGACY_ADV_WIRE_LEN
    } else {
        MAX_DATA_WIRE_LEN
    };
    if wire_len > maximum_wire_len {
        return ReceiveLengthPolicy::UnsupportedCapability;
    }
    ReceiveLengthPolicy::Accepted { wire_len }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_enabled_legacy_and_pre_dle_limits() {
        assert_eq!(
            classify_received_length(true, MAX_LEGACY_ADV_PAYLOAD_LEN),
            ReceiveLengthPolicy::Accepted {
                wire_len: MAX_LEGACY_ADV_WIRE_LEN
            }
        );
        assert_eq!(
            classify_received_length(false, CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN),
            ReceiveLengthPolicy::Accepted {
                wire_len: MAX_DATA_WIRE_LEN
            }
        );
    }

    #[test]
    fn distinguishes_unsupported_protocol_from_physical_overflow() {
        assert_eq!(
            classify_received_length(true, MAX_LEGACY_ADV_PAYLOAD_LEN + 1),
            ReceiveLengthPolicy::UnsupportedCapability
        );
        assert_eq!(
            classify_received_length(false, CONNECTION_RX_MAX_WIRE_PAYLOAD_LEN + 1),
            ReceiveLengthPolicy::UnsupportedCapability
        );
        assert_eq!(
            classify_received_length(true, RF_DMA_BUFFER_LEN - LL_HEADER_LEN + 1),
            ReceiveLengthPolicy::PhysicalDmaOverflow
        );
        assert_eq!(
            classify_received_length(false, u8::MAX as usize),
            ReceiveLengthPolicy::PhysicalDmaOverflow
        );
    }
}

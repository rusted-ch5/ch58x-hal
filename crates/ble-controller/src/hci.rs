//! Plaintext Peripheral HCI capability and input validation.
//!
//! This module contains no queues, registers, Host stack, or security engine.

pub const LE_FEATURES: [u8; 8] = [0x06, 0, 0, 0, 0, 0, 0, 0];
pub const ACL_PAYLOAD_LEN: usize = 27;
pub const ACL_CREDITS: u8 = 5;

/// Legacy connectable advertising parameters supported by this controller.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdvertisingParameters {
    pub interval_us: u64,
    pub channels: [bool; 3],
}

impl AdvertisingParameters {
    /// Accept only ADV_IND with a random address and no filter list.
    /// Validation completes before the caller changes live advertiser state.
    pub fn parse(params: &[u8]) -> Option<Self> {
        if params.len() != 15 {
            return None;
        }
        let min = u16::from_le_bytes([params[0], params[1]]);
        let max = u16::from_le_bytes([params[2], params[3]]);
        if !(0x20..=0x4000).contains(&min)
            || !(min..=0x4000).contains(&max)
            || params[4] != 0
            || params[5] != 1
            || params[6] > 1
            || params[13] == 0
            || params[13] & !7 != 0
            || params[14] != 0
        {
            return None;
        }
        Some(Self {
            interval_us: u64::from(min) * 625,
            channels: [
                params[13] & 1 != 0,
                params[13] & 2 != 0,
                params[13] & 4 != 0,
            ],
        })
    }
}

/// Apply the two event-mask namespaces. HCI credit/completion events bypass
/// event masking so a Host cannot deadlock its command or ACL pipeline.
pub fn event_enabled(mask: u64, le_mask: u64, code: u8, params: &[u8]) -> bool {
    if matches!(code, 0x0e | 0x0f | 0x13) {
        return true;
    }
    if code == 0 || code > 64 || mask & (1u64 << (code - 1)) == 0 {
        return false;
    }
    if code == 0x3e {
        return params.first().is_some_and(|&subevent| {
            subevent != 0 && subevent <= 64 && le_mask & (1u64 << (subevent - 1)) != 0
        });
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reject_unsupported_modes_and_malformed_parameters() {
        let mut params = [0u8; 15];
        params[..4].copy_from_slice(&[0x20, 0, 0x40, 0]);
        params[5] = 1;
        params[13] = 7;
        assert_eq!(
            AdvertisingParameters::parse(&params).unwrap().interval_us,
            20_000
        );
        for len in 0..15 {
            assert!(AdvertisingParameters::parse(&params[..len]).is_none());
        }
        for (offset, value) in [
            (0, 0x1f),
            (3, 0x41),
            (4, 1),
            (5, 2),
            (6, 2),
            (13, 0),
            (13, 8),
            (14, 1),
        ] {
            let mut invalid = params;
            invalid[offset] = value;
            assert!(
                AdvertisingParameters::parse(&invalid).is_none(),
                "offset {offset}"
            );
        }
    }

    #[test]
    fn masks_cannot_suppress_credit_events_or_cross_namespaces() {
        for code in [0x0e, 0x0f, 0x13] {
            assert!(event_enabled(0, 0, code, &[]));
        }
        assert!(!event_enabled(0, u64::MAX, 0x3e, &[1]));
        assert!(!event_enabled(u64::MAX, 0, 0x3e, &[1]));
        assert!(event_enabled(u64::MAX, 1, 0x3e, &[1]));
        assert!(!event_enabled(u64::MAX, 1, 0x3e, &[3]));
        for subevent in [0, 65, 255] {
            assert!(!event_enabled(u64::MAX, u64::MAX, 0x3e, &[subevent]));
        }
        assert!(!event_enabled(u64::MAX, u64::MAX, 0x3e, &[]));
        assert!(!event_enabled(u64::MAX, u64::MAX, 0, &[]));
    }

    #[test]
    fn capabilities_do_not_advertise_crypto_or_dle() {
        assert_eq!(LE_FEATURES[0] & 1, 0);
        assert_eq!(LE_FEATURES[0] & 0x20, 0);
        assert_eq!(ACL_PAYLOAD_LEN, 27);
        assert_eq!(ACL_CREDITS, 5);
    }
}

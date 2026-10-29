//! Legacy advertising PDU construction.

use crate::Error;
use crate::link_layer::{
    connection_timing_is_valid, data_access_address_is_valid, data_channel_map_is_valid,
};

pub const MAX_LEGACY_ADV_DATA_LEN: usize = 31;
pub const MAX_LEGACY_ADV_PDU_LEN: usize = 2 + 6 + MAX_LEGACY_ADV_DATA_LEN;
pub const CONNECT_IND_PAYLOAD_LEN: usize = 34;

pub const AD_TYPE_FLAGS: u8 = 0x01;
pub const AD_TYPE_COMPLETE_UUID16_LIST: u8 = 0x03;
pub const AD_TYPE_COMPLETE_LOCAL_NAME: u8 = 0x09;
pub const AD_TYPE_APPEARANCE: u8 = 0x19;

pub const FLAG_LE_GENERAL_DISCOVERABLE: u8 = 1 << 1;
pub const FLAG_BR_EDR_NOT_SUPPORTED: u8 = 1 << 2;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AddressKind {
    Public,
    Random,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceAddress {
    /// Address octets in Link Layer transmission order (least significant
    /// address octet first).
    pub bytes: [u8; 6],
    pub kind: AddressKind,
}

impl DeviceAddress {
    pub const fn public(bytes: [u8; 6]) -> Self {
        Self {
            bytes,
            kind: AddressKind::Public,
        }
    }

    pub const fn random_static(bytes: [u8; 6]) -> Result<Self, Error> {
        let random_part_all_zero = bytes[0] == 0
            && bytes[1] == 0
            && bytes[2] == 0
            && bytes[3] == 0
            && bytes[4] == 0
            && bytes[5] & 0x3f == 0;
        let random_part_all_one = bytes[0] == 0xff
            && bytes[1] == 0xff
            && bytes[2] == 0xff
            && bytes[3] == 0xff
            && bytes[4] == 0xff
            && bytes[5] & 0x3f == 0x3f;
        if bytes[5] & 0xc0 != 0xc0 || random_part_all_zero || random_part_all_one {
            return Err(Error::InvalidPacket);
        }
        Ok(Self {
            bytes,
            kind: AddressKind::Random,
        })
    }
}

/// A validated legacy advertising PDU, retaining the raw address type needed
/// by HCI scan reports and by a later CONNECT_IND.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LegacyAdvertisement<'a> {
    /// HCI legacy advertising event type: ADV_IND=0, ADV_DIRECT_IND=1,
    /// ADV_SCAN_IND=2, ADV_NONCONN_IND=3, SCAN_RSP=4.
    pub event_type: u8,
    pub address: DeviceAddress,
    pub data: &'a [u8],
}

/// Parse a complete, CRC-validated legacy advertising-channel PDU.
///
/// The radio owns CRC verification. This parser deliberately performs no
/// duplicate filtering and exposes the address bytes in Link Layer order.
pub fn parse_legacy_advertisement(packet: &[u8]) -> Result<LegacyAdvertisement<'_>, Error> {
    if packet.len() < 8 {
        return Err(Error::InvalidPacket);
    }
    let payload_len = usize::from(packet[1] & 0x3f);
    if !(6..=37).contains(&payload_len) || packet.len() != payload_len + 2 {
        return Err(Error::InvalidPacket);
    }
    let pdu_type = packet[0] & 0x0f;
    let (event_type, data_offset) = match pdu_type {
        0x00 => (0, 8),                       // ADV_IND
        0x01 if payload_len == 12 => (1, 14), // ADV_DIRECT_IND
        0x02 => (3, 8),                       // ADV_NONCONN_IND
        0x04 => (4, 8),                       // SCAN_RSP
        0x06 => (2, 8),                       // ADV_SCAN_IND
        _ => return Err(Error::InvalidPacket),
    };
    let address = DeviceAddress {
        bytes: packet[2..8].try_into().unwrap(),
        kind: if packet[0] & (1 << 6) != 0 {
            AddressKind::Random
        } else {
            AddressKind::Public
        },
    };
    Ok(LegacyAdvertisement {
        event_type,
        address,
        data: &packet[data_offset..payload_len + 2],
    })
}

pub struct AdvertisingData {
    bytes: [u8; MAX_LEGACY_ADV_DATA_LEN],
    len: usize,
}

impl AdvertisingData {
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_LEGACY_ADV_DATA_LEN],
            len: 0,
        }
    }

    pub fn push(&mut self, ad_type: u8, value: &[u8]) -> Result<(), Error> {
        let field_len = value.len().checked_add(2).ok_or(Error::InvalidPacket)?;
        let end = self
            .len
            .checked_add(field_len)
            .ok_or(Error::InvalidPacket)?;
        if value.len() > u8::MAX as usize - 1 || end > self.bytes.len() {
            return Err(Error::InvalidPacket);
        }

        self.bytes[self.len] = (value.len() + 1) as u8;
        self.bytes[self.len + 1] = ad_type;
        self.bytes[self.len + 2..end].copy_from_slice(value);
        self.len = end;
        Ok(())
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Default for AdvertisingData {
    fn default() -> Self {
        Self::new()
    }
}

/// Builds a connectable, scannable legacy `ADV_IND` PDU.
///
/// Channel Selection Algorithm #2 is intentionally not advertised until the
/// controller implements it. The result contains only Header + Payload; the
/// radio backend supplies access address, CRC and whitening.
pub fn build_adv_ind<'a>(
    address: DeviceAddress,
    data: &AdvertisingData,
    output: &'a mut [u8; MAX_LEGACY_ADV_PDU_LEN],
) -> Result<&'a [u8], Error> {
    let payload_len = 6 + data.len;
    let pdu_len = 2 + payload_len;

    output[0] = match address.kind {
        AddressKind::Public => 0x00,
        AddressKind::Random => 1 << 6,
    };
    output[1] = payload_len as u8;
    output[2..8].copy_from_slice(&address.bytes);
    output[8..pdu_len].copy_from_slice(data.as_bytes());
    Ok(&output[..pdu_len])
}

/// Builds a legacy `SCAN_RSP` PDU for the same advertiser address.
pub fn build_scan_rsp<'a>(
    address: DeviceAddress,
    data: &AdvertisingData,
    output: &'a mut [u8; MAX_LEGACY_ADV_PDU_LEN],
) -> Result<&'a [u8], Error> {
    let payload_len = 6 + data.len;
    let pdu_len = 2 + payload_len;
    output[0] = 0x04
        | match address.kind {
            AddressKind::Public => 0,
            AddressKind::Random => 1 << 6,
        };
    output[1] = payload_len as u8;
    output[2..8].copy_from_slice(&address.bytes);
    output[8..pdu_len].copy_from_slice(data.as_bytes());
    Ok(&output[..pdu_len])
}

/// Returns whether a received legacy advertising PDU is a SCAN_REQ directed
/// at `address`. The scanner address is deliberately not interpreted here.
pub fn is_scan_request(packet: &[u8], address: DeviceAddress) -> bool {
    packet.len() >= 14
        && packet[0] & 0x0f == 0x03
        && packet[1] & 0x3f == 12
        && packet[8..14] == address.bytes
}

/// Parsed CONNECT_IND fields needed to enter the first connection event.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectInd {
    pub initiator_address: [u8; 6],
    pub initiator_address_random: bool,
    pub access_address: u32,
    pub crc_init: u32,
    pub window_size: u8,
    pub window_offset: u16,
    pub interval: u16,
    pub latency: u16,
    pub supervision_timeout: u16,
    pub channel_map: [u8; 5],
    pub hop_increment: u8,
    pub sleep_clock_accuracy: u8,
}

/// Build a complete legacy CONNECT_IND PDU from already validated connection
/// parameters. The caller must transmit it exactly one IFS after the ADV_IND
/// it responds to.
pub fn build_connect_ind(
    initiator: DeviceAddress,
    advertiser: DeviceAddress,
    connection: ConnectInd,
    output: &mut [u8; 2 + CONNECT_IND_PAYLOAD_LEN],
) -> Result<&[u8], Error> {
    if connection.initiator_address != initiator.bytes
        || connection.initiator_address_random != matches!(initiator.kind, AddressKind::Random)
        || !data_access_address_is_valid(connection.access_address, false)
        || !(5..=16).contains(&connection.hop_increment)
        || connection.sleep_clock_accuracy > 7
        || !connection_timing_is_valid(
            connection.window_size,
            connection.window_offset,
            connection.interval,
            connection.latency,
            connection.supervision_timeout,
        )
        || !data_channel_map_is_valid(connection.channel_map)
    {
        return Err(Error::InvalidPacket);
    }

    output.fill(0);
    output[0] =
        0x05 | if matches!(initiator.kind, AddressKind::Random) {
            1 << 6
        } else {
            0
        } | if matches!(advertiser.kind, AddressKind::Random) {
            1 << 7
        } else {
            0
        };
    output[1] = CONNECT_IND_PAYLOAD_LEN as u8;
    output[2..8].copy_from_slice(&initiator.bytes);
    output[8..14].copy_from_slice(&advertiser.bytes);
    output[14..18].copy_from_slice(&connection.access_address.to_le_bytes());
    output[18] = connection.crc_init as u8;
    output[19] = (connection.crc_init >> 8) as u8;
    output[20] = (connection.crc_init >> 16) as u8;
    output[21] = connection.window_size;
    output[22..24].copy_from_slice(&connection.window_offset.to_le_bytes());
    output[24..26].copy_from_slice(&connection.interval.to_le_bytes());
    output[26..28].copy_from_slice(&connection.latency.to_le_bytes());
    output[28..30].copy_from_slice(&connection.supervision_timeout.to_le_bytes());
    output[30..35].copy_from_slice(&connection.channel_map);
    output[35] = connection.hop_increment | (connection.sleep_clock_accuracy << 5);
    Ok(output)
}

/// Exact reason a legacy connection request was not accepted.
///
/// Zephyr's controller keeps the radio-completion/address gate separate from
/// connection parameter setup. Preserve the same observability here instead
/// of collapsing every directed-but-invalid request into `InvalidPacket`.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConnectIndParseError {
    NotConnectInd = 1,
    InvalidLength = 2,
    DifferentAdvertiser = 3,
    InvalidAccessAddress = 4,
    InvalidHopIncrement = 5,
    InvalidTiming = 6,
    InvalidChannelMap = 7,
}

/// Parses a CONNECT_IND while retaining the precise rejection boundary.
pub fn parse_connect_ind_detailed(
    packet: &[u8],
    address: DeviceAddress,
) -> Result<ConnectInd, ConnectIndParseError> {
    if packet.len() < 2 + CONNECT_IND_PAYLOAD_LEN || packet[0] & 0x0f != 0x05 {
        return Err(ConnectIndParseError::NotConnectInd);
    }
    if packet[1] & 0x3f != CONNECT_IND_PAYLOAD_LEN as u8 {
        return Err(ConnectIndParseError::InvalidLength);
    }
    let payload = &packet[2..2 + CONNECT_IND_PAYLOAD_LEN];
    if payload[6..12] != address.bytes {
        return Err(ConnectIndParseError::DifferentAdvertiser);
    }
    let access_address = u32::from_le_bytes(payload[12..16].try_into().unwrap());
    let crc_init =
        u32::from(payload[16]) | (u32::from(payload[17]) << 8) | (u32::from(payload[18]) << 16);
    let hop_sca = payload[33];
    if !data_access_address_is_valid(access_address, false) {
        return Err(ConnectIndParseError::InvalidAccessAddress);
    }
    if !(5..=16).contains(&(hop_sca & 0x1f)) {
        return Err(ConnectIndParseError::InvalidHopIncrement);
    }
    let connection = ConnectInd {
        initiator_address: payload[..6].try_into().unwrap(),
        initiator_address_random: packet[0] & (1 << 6) != 0,
        access_address,
        crc_init,
        window_size: payload[19],
        window_offset: u16::from_le_bytes(payload[20..22].try_into().unwrap()),
        interval: u16::from_le_bytes(payload[22..24].try_into().unwrap()),
        latency: u16::from_le_bytes(payload[24..26].try_into().unwrap()),
        supervision_timeout: u16::from_le_bytes(payload[26..28].try_into().unwrap()),
        channel_map: payload[28..33].try_into().unwrap(),
        hop_increment: hop_sca & 0x1f,
        sleep_clock_accuracy: hop_sca >> 5,
    };
    if !connection_timing_is_valid(
        connection.window_size,
        connection.window_offset,
        connection.interval,
        connection.latency,
        connection.supervision_timeout,
    ) {
        return Err(ConnectIndParseError::InvalidTiming);
    }
    if !data_channel_map_is_valid(connection.channel_map) {
        return Err(ConnectIndParseError::InvalidChannelMap);
    }
    Ok(connection)
}

/// Parses a CONNECT_IND directed at `address`.
pub fn parse_connect_ind(
    packet: &[u8],
    address: DeviceAddress,
) -> Result<Option<ConnectInd>, Error> {
    match parse_connect_ind_detailed(packet, address) {
        Ok(connection) => Ok(Some(connection)),
        Err(ConnectIndParseError::NotConnectInd | ConnectIndParseError::DifferentAdvertiser) => {
            Ok(None)
        }
        Err(_) => Err(Error::InvalidPacket),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_connect_ind(address: DeviceAddress) -> [u8; 36] {
        let mut pdu = [0u8; 36];
        pdu[0] = 0x05;
        pdu[1] = 34;
        pdu[2..8].copy_from_slice(&[6, 5, 4, 3, 2, 1]);
        pdu[8..14].copy_from_slice(&address.bytes);
        pdu[14..18].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        pdu[18..21].copy_from_slice(&[0x56, 0x34, 0x12]);
        pdu[21] = 2;
        pdu[22..24].copy_from_slice(&1u16.to_le_bytes());
        pdu[24..26].copy_from_slice(&24u16.to_le_bytes());
        pdu[26..28].copy_from_slice(&0u16.to_le_bytes());
        pdu[28..30].copy_from_slice(&200u16.to_le_bytes());
        pdu[30..35].copy_from_slice(&[0xff, 0xff, 0xff, 0xff, 0x1f]);
        pdu[35] = 5;
        pdu
    }

    #[test]
    fn parses_legacy_advertising_address_type_and_data() {
        let packet = [0x40, 9, 1, 2, 3, 4, 5, 0xc6, 2, 1, 6];
        let parsed = parse_legacy_advertisement(&packet).unwrap();
        assert_eq!(parsed.event_type, 0);
        assert_eq!(parsed.address.bytes, [1, 2, 3, 4, 5, 0xc6]);
        assert_eq!(parsed.address.kind, AddressKind::Random);
        assert_eq!(parsed.data, [2, 1, 6]);

        let scan_rsp = [0x04, 8, 6, 5, 4, 3, 2, 1, 1, 9];
        assert_eq!(parse_legacy_advertisement(&scan_rsp).unwrap().event_type, 4);
        assert!(parse_legacy_advertisement(&packet[..10]).is_err());
    }

    #[test]
    fn connect_ind_builder_round_trips_through_peripheral_parser() {
        let advertiser = DeviceAddress::random_static([1, 2, 3, 4, 5, 0xc6]).unwrap();
        let initiator = DeviceAddress::public([6, 5, 4, 3, 2, 1]);
        let parsed =
            parse_connect_ind_detailed(&valid_connect_ind(advertiser), advertiser).unwrap();
        let mut output = [0; 36];
        let built = build_connect_ind(initiator, advertiser, parsed, &mut output).unwrap();
        assert_eq!(built[0], 0x85);
        assert_eq!(
            parse_connect_ind_detailed(built, advertiser).unwrap(),
            parsed
        );
    }

    #[test]
    fn builds_named_hid_keyboard_advertisement() {
        let mut data = AdvertisingData::new();
        data.push(
            AD_TYPE_FLAGS,
            &[FLAG_LE_GENERAL_DISCOVERABLE | FLAG_BR_EDR_NOT_SUPPORTED],
        )
        .unwrap();
        data.push(AD_TYPE_APPEARANCE, &0x03c1u16.to_le_bytes())
            .unwrap();
        data.push(AD_TYPE_COMPLETE_UUID16_LIST, &[0x12, 0x18, 0x0f, 0x18])
            .unwrap();
        data.push(AD_TYPE_COMPLETE_LOCAL_NAME, b"rmk-ch582m")
            .unwrap();

        assert_eq!(data.as_bytes().len(), 25);
        let address = DeviceAddress::random_static([1, 2, 3, 4, 5, 0xc6]).unwrap();
        let mut output = [0; MAX_LEGACY_ADV_PDU_LEN];
        let pdu = build_adv_ind(address, &data, &mut output).unwrap();
        assert_eq!(pdu[0], 0x40);
        assert_eq!(pdu[1], 31);
        assert_eq!(&pdu[2..8], &[1, 2, 3, 4, 5, 0xc6]);
        assert!(pdu.ends_with(b"rmk-ch582m"));
    }

    #[test]
    fn rejects_non_static_random_address() {
        assert_eq!(
            DeviceAddress::random_static([1, 2, 3, 4, 5, 0x86]),
            Err(Error::InvalidPacket)
        );
        assert_eq!(
            DeviceAddress::random_static([0, 0, 0, 0, 0, 0xc0]),
            Err(Error::InvalidPacket)
        );
        assert_eq!(
            DeviceAddress::random_static([0xff; 6]),
            Err(Error::InvalidPacket)
        );
    }

    #[test]
    fn rejects_advertising_data_overflow() {
        let mut data = AdvertisingData::new();
        assert_eq!(
            data.push(AD_TYPE_COMPLETE_LOCAL_NAME, &[0; 31]),
            Err(Error::InvalidPacket)
        );
    }

    #[test]
    fn parses_connect_ind_for_this_advertiser() {
        let address = DeviceAddress::random_static([1, 2, 3, 4, 5, 0xc6]).unwrap();
        let pdu = valid_connect_ind(address);

        let parsed = parse_connect_ind(&pdu, address).unwrap().unwrap();
        assert_eq!(parsed.access_address, 0x1234_5678);
        assert_eq!(parsed.crc_init, 0x123456);
        assert_eq!(parsed.interval, 24);
        assert_eq!(parsed.hop_increment, 5);
    }

    #[test]
    fn rejects_connect_ind_with_invalid_timing_or_channel_map() {
        let address = DeviceAddress::random_static([1, 2, 3, 4, 5, 0xc6]).unwrap();

        let mut invalid_window = valid_connect_ind(address);
        invalid_window[21] = 0;
        assert_eq!(
            parse_connect_ind(&invalid_window, address),
            Err(Error::InvalidPacket)
        );

        let mut invalid_timeout = valid_connect_ind(address);
        // 100 ms is not strictly greater than 2 * 50 ms.
        invalid_timeout[24..26].copy_from_slice(&40u16.to_le_bytes());
        invalid_timeout[28..30].copy_from_slice(&10u16.to_le_bytes());
        assert_eq!(
            parse_connect_ind(&invalid_timeout, address),
            Err(Error::InvalidPacket)
        );

        let mut invalid_map = valid_connect_ind(address);
        invalid_map[30..35].copy_from_slice(&[1, 0, 0, 0, 0]);
        assert_eq!(
            parse_connect_ind(&invalid_map, address),
            Err(Error::InvalidPacket)
        );

        let mut invalid_access_address = valid_connect_ind(address);
        invalid_access_address[14..18].copy_from_slice(&0x1111_1111u32.to_le_bytes());
        assert_eq!(
            parse_connect_ind(&invalid_access_address, address),
            Err(Error::InvalidPacket)
        );

        assert_eq!(
            parse_connect_ind_detailed(&invalid_window, address),
            Err(ConnectIndParseError::InvalidTiming)
        );
        assert_eq!(
            parse_connect_ind_detailed(&invalid_map, address),
            Err(ConnectIndParseError::InvalidChannelMap)
        );
        assert_eq!(
            parse_connect_ind_detailed(&invalid_access_address, address),
            Err(ConnectIndParseError::InvalidAccessAddress)
        );
    }
}

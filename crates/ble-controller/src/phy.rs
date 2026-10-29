#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Phy {
    Le1M,
    Le2M,
    LeCodedS2,
    LeCodedS8,
}

#[derive(Debug)]
pub struct ReceivedPacket<'a> {
    pub payload: &'a [u8],
    pub rssi_dbm: i8,
    pub crc_ok: bool,
    pub timestamp_us: u64,
}

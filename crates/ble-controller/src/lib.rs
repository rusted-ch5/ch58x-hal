#![no_std]
#![allow(async_fn_in_trait)]

//! Portable Bluetooth LE controller core.
//!
//! Chip backends implement [`BleRadio`]. This crate owns protocol-defined PHY
//! transforms and Link Layer state; it contains no WCH-specific registers.

pub mod advertising;
pub mod channel;
pub mod crc;
pub mod hci;
pub mod link_layer;
pub mod llcp;
pub mod phy;
pub mod rf_calibration;
pub mod whitening;

pub use channel::Channel;
pub use phy::{Phy, ReceivedPacket};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Busy,
    Timeout,
    InvalidChannel,
    InvalidPacket,
    Hardware,
}

/// Hardware-independent BLE radio contract used by the Link Layer.
pub trait BleRadio {
    async fn transmit(&mut self, channel: Channel, packet: &[u8]) -> Result<(), Error>;
    async fn receive<'a>(
        &mut self,
        channel: Channel,
        timeout: embassy_time::Duration,
        buffer: &'a mut [u8],
    ) -> Result<ReceivedPacket<'a>, Error>;
    fn set_access_address(&mut self, access_address: u32);
    fn set_crc_init(&mut self, crc_init: u32);
    fn set_phy(&mut self, phy: Phy) -> Result<(), Error>;
}

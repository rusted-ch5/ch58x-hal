//! In-process standard HCI boundary for the register-level CH58x controller.
//!
//! Trouble uses [`Controller`] exactly like an external HCI controller. The
//! transport below serializes commands and ACL packets into bounded static
//! queues; [`crate::link_layer::Runner`] owns the radio and consumes them.

use bt_hci::controller::ExternalController;
use core::convert::Infallible;
use portable_atomic::{AtomicU64, Ordering};
static EVENT_MASK: AtomicU64 = AtomicU64::new(u64::MAX);
static LE_EVENT_MASK: AtomicU64 = AtomicU64::new(0x1f);
pub(crate) fn set_event_mask(mask: u64) {
    EVENT_MASK.store(mask, Ordering::Release);
}
pub(crate) fn set_le_event_mask(mask: u64) {
    LE_EVENT_MASK.store(mask, Ordering::Release);
}

use bt_hci::transport::{PacketToController, PacketToHost, Transport as TransportTrait};
use bt_hci::{FromHciBytesError, PacketKind, ReadHciError};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// This controller advertises the Core-mandated initial 27-byte ACL data
// length and does not implement DLE yet. Commands used by Trouble's legacy
// advertiser are at most 35 bytes, while ACL frames are at most 31 bytes
// including their HCI header. The bounded transport does not allocate.
pub(crate) const HCI_PACKET_CAPACITY: usize = 64;

#[derive(Clone, Copy)]
pub(crate) struct Packet {
    pub kind: PacketKind,
    pub len: usize,
    pub bytes: [u8; HCI_PACKET_CAPACITY],
}

impl Packet {
    pub const fn empty(kind: PacketKind) -> Self {
        Self {
            kind,
            len: 0,
            bytes: [0; HCI_PACKET_CAPACITY],
        }
    }

    pub fn from_bytes(kind: PacketKind, data: &[u8]) -> Result<Self, Error> {
        if data.len() > HCI_PACKET_CAPACITY {
            return Err(Error::PacketTooLarge);
        }
        let mut packet = Self::empty(kind);
        packet.bytes[..data.len()].copy_from_slice(data);
        packet.len = data.len();
        Ok(packet)
    }
}

// Bounded queues provide back-pressure without heap allocation.
const HOST_TO_CONTROLLER_CAPACITY: usize = 1;

static HOST_TO_CONTROLLER: Channel<CriticalSectionRawMutex, Packet, HOST_TO_CONTROLLER_CAPACITY> =
    Channel::new();

const CONTROLLER_TO_HOST_CAPACITY: usize = 2;
static CONTROLLER_TO_HOST: Channel<CriticalSectionRawMutex, Packet, CONTROLLER_TO_HOST_CAPACITY> =
    Channel::new();

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    PacketTooLarge,
    MalformedPacket,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::PacketTooLarge => f.write_str("HCI packet too large"),
            Self::MalformedPacket => f.write_str("malformed HCI packet"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::PacketTooLarge => embedded_io::ErrorKind::OutOfMemory,
            Self::MalformedPacket => embedded_io::ErrorKind::InvalidData,
        }
    }
}

impl From<FromHciBytesError> for Error {
    fn from(_: FromHciBytesError) -> Self {
        Self::MalformedPacket
    }
}

impl From<ReadHciError<Infallible>> for Error {
    fn from(_: ReadHciError<Infallible>) -> Self {
        Self::MalformedPacket
    }
}

/// Packet transport used by `bt-hci`'s command dispatcher.
pub struct HciTransport {
    _private: (),
}

impl HciTransport {
    /// Creates the unique raw HCI transport for controller-level fixtures.
    pub fn new(_token: crate::ControllerToken) -> Self {
        Self { _private: () }
    }
}

impl embedded_io::ErrorType for HciTransport {
    type Error = Error;
}

impl TransportTrait for HciTransport {
    async fn read<'a, P: PacketToHost<'a>>(&self, output: &'a mut [u8]) -> Result<P, Self::Error> {
        let packet = CONTROLLER_TO_HOST.receive().await;

        if packet.len > output.len() {
            return Err(Error::PacketTooLarge);
        }
        let mut input = &packet.bytes[..packet.len];
        let packet = P::read_hci(packet.kind, &mut input, output)?;
        if !input.is_empty() {
            return Err(Error::MalformedPacket);
        }
        Ok(packet)
    }

    async fn write<T: PacketToController>(&self, value: &T) -> Result<(), Self::Error> {
        if value.size() > HCI_PACKET_CAPACITY {
            return Err(Error::PacketTooLarge);
        }
        let mut packet = Packet::empty(T::KIND);
        let size = value.size();
        let mut destination = &mut packet.bytes[..size];
        value
            .write_hci(&mut destination)
            .map_err(|_| Error::PacketTooLarge)?;
        packet.len = size;

        HOST_TO_CONTROLLER.send(packet).await;
        Ok(())
    }
}

/// Standard HCI controller consumed by a Bluetooth Host.
pub type Controller = ExternalController<HciTransport, 4>;

/// Creates the unique HCI facade. The paired Link Layer runner must be polled.
pub fn new(_token: crate::ControllerToken) -> Controller {
    ExternalController::new(HciTransport::new(_token))
}

pub(crate) async fn receive_host_packet() -> Packet {
    HOST_TO_CONTROLLER.receive().await
}

pub(crate) fn try_receive_host_packet() -> Option<Packet> {
    let packet = HOST_TO_CONTROLLER.try_receive().ok()?;
    Some(packet)
}

/// Return the channel future without duplicating its packet storage.
///
/// An `async fn` taking an owned `Packet` and then awaiting `Channel::send`
/// makes rustc reserve separate coroutine slots for the argument, its move
/// state, and the channel future's owned message. Returning the send future
/// keeps the same bounded back-pressure semantics without tripling that
/// storage in every Controller-to-Host event path.
pub(crate) fn send_controller_packet(packet: Packet) -> impl core::future::Future<Output = ()> {
    CONTROLLER_TO_HOST.send(packet)
}

pub(crate) async fn event(event_code: u8, parameters: &[u8]) -> Result<(), Error> {
    if parameters.len() > HCI_PACKET_CAPACITY - 2 {
        return Err(Error::PacketTooLarge);
    }
    if !ble_controller::hci::event_enabled(
        EVENT_MASK.load(Ordering::Acquire),
        LE_EVENT_MASK.load(Ordering::Acquire),
        event_code,
        parameters,
    ) {
        return Ok(());
    }
    let mut packet = Packet::empty(PacketKind::Event);
    packet.bytes[0] = event_code;
    packet.bytes[1] = parameters.len() as u8;
    packet.bytes[2..2 + parameters.len()].copy_from_slice(parameters);
    packet.len = 2 + parameters.len();
    send_controller_packet(packet).await;
    Ok(())
}

pub(crate) async fn command_complete(opcode: u16, status: u8, return_data: &[u8]) {
    let parameter_len = 4usize.saturating_add(return_data.len());
    let mut bytes = [0u8; HCI_PACKET_CAPACITY];
    bytes[0] = 0x0e;
    bytes[1] = parameter_len as u8;
    bytes[2] = 1;
    bytes[3..5].copy_from_slice(&opcode.to_le_bytes());
    bytes[5] = status;
    bytes[6..6 + return_data.len()].copy_from_slice(return_data);
    send_controller_packet(
        Packet::from_bytes(PacketKind::Event, &bytes[..6 + return_data.len()]).unwrap(),
    )
    .await;
}

pub(crate) async fn command_status(opcode: u16, status: u8) {
    send_controller_packet(
        Packet::from_bytes(
            PacketKind::Event,
            &[0x0f, 0x04, status, 0x01, opcode as u8, (opcode >> 8) as u8],
        )
        .unwrap(),
    )
    .await;
}

pub(crate) async fn send_acl(handle_and_flags: u16, payload: &[u8]) -> Result<(), Error> {
    let mut packet = Packet::empty(PacketKind::AclData);
    let len = payload.len();
    if len + 4 > packet.bytes.len() {
        return Err(Error::PacketTooLarge);
    }
    packet.bytes[..2].copy_from_slice(&handle_and_flags.to_le_bytes());
    packet.bytes[2..4].copy_from_slice(&(len as u16).to_le_bytes());
    packet.bytes[4..4 + len].copy_from_slice(payload);
    packet.len = len + 4;

    send_controller_packet(packet).await;
    Ok(())
}

pub(crate) async fn completed_acl(handle: u16, count: u16) {
    let mut params = [0u8; 5];
    params[0] = 1;
    params[1..3].copy_from_slice(&handle.to_le_bytes());
    params[3..5].copy_from_slice(&count.to_le_bytes());
    let _ = event(0x13, &params).await;
}

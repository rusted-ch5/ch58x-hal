//! CH585 Link Layer event scheduling and connection ownership.

mod hci;

use ble_controller::advertising::{
    AdvertisingData, ConnectInd, DeviceAddress, MAX_LEGACY_ADV_PDU_LEN, build_adv_ind,
    build_scan_rsp, is_scan_request, parse_connect_ind_detailed,
};

use ble_controller::link_layer::{
    ADV_ACCESS_ADDRESS, ADV_CRC_INIT, ChannelMapInd, ChannelSelection1, ConnectionParamPdu,
    ConnectionUpdateInd, ControlPduError, DATA_HEADER_NESN, DATA_HEADER_SEQUENCE_MASK,
    DATA_HEADER_SN, InstantProcedure, acknowledges_transmit_owner,
    connection_parameter_range_is_valid, connection_update_matches_request,
    procedure_collision_error, window_widening_us,
};

use ble_controller::llcp::{
    ProcedureContext, ProcedureError, ProcedureInitiator, ProcedureKind, ProcedureManager,
    ProcedurePhase, ProcedureToken,
};
use ble_controller::{BleRadio, Channel, Error as RadioError};
use bt_hci::PacketKind;
use embassy_futures::select::{Either, select};
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};

use portable_atomic::{AtomicBool, Ordering};

use rand_chacha::ChaCha20Rng;
use rand_core::RngCore;

use crate::controller::{self, Packet};

use crate::radio::Radio;

const HCI_SUCCESS: u8 = 0x00;
const HCI_UNKNOWN_COMMAND: u8 = 0x01;
const HCI_COMMAND_DISALLOWED: u8 = 0x0c;
const HCI_INVALID_PARAMETERS: u8 = 0x12;
const HCI_CONTROLLER_BUSY: u8 = 0x3a;

const CONNECTION_HANDLE: u16 = 1;

// Until the Data Length Update procedure is implemented end-to-end, the
// controller must use the Core-mandated initial LE 1M data length. HCI reports
// the same value, so Trouble fragments larger L2CAP SDUs instead of handing a
// pre-DLE 247-octet PDU to the Link Layer.
const MAX_DATA_PDU_PAYLOAD: usize = ble_controller::hci::ACL_PAYLOAD_LEN;
const MAX_DATA_PDU_WIRE_LEN: usize = 2 + MAX_DATA_PDU_PAYLOAD;

const ACL_QUEUE_CAPACITY: usize = 4;

const LLID_CONTINUATION: u8 = 0x01;
const LLID_START: u8 = 0x02;

fn refresh_transmit_sequence_header(header: &mut u8, transmit_sn: u8, expected_sn: u8) {
    *header = (*header & !DATA_HEADER_SEQUENCE_MASK)
        | ((expected_sn & 1) * DATA_HEADER_NESN)
        | ((transmit_sn & 1) * DATA_HEADER_SN);
}
const LLID_CONTROL: u8 = 0x03;
// HCI ACL Packet_Boundary_Flag values live in bits 12..13 of the combined
// handle field. Controller-to-Host first fragments must use First Flushable
// (`0b10`); `0b00` is First Non-Flushable and is only valid Host-to-Controller.
const HCI_PB_CONTINUING: u16 = 1 << 12;
const HCI_PB_FIRST_FLUSHABLE: u16 = 2 << 12;

type ControllerRng = ChaCha20Rng;

fn controller_rng(random: ChaCha20Rng) -> ControllerRng {
    random
}

// Wake before the Core-defined receive window so Embassy and the CH585 RF
// registers are settled at its leading edge. Clock uncertainty itself is
// accumulated separately from the peer/local SCA instead of using a fixed
// early margin.
// Reserve a conservative 1-ms executor/setup budget independently of SCA.
const RX_SETUP_LEAD_US: u64 = 1_000;

// The dev board has a 32.768-kHz crystal. This deliberately claims only the
// 50-ppm class until a frequency-counter measurement justifies a tighter bin.
const LOCAL_SLEEP_CLOCK_PPM: u16 = 50;
// Longest plaintext LE 1M data packet before Data Length Update.
const MAX_DATA_PACKET_AIRTIME_US: u64 = ((1 + 4 + 2 + MAX_DATA_PDU_PAYLOAD + 3) * 8) as u64;
const PROCEDURE_RESPONSE_TIMEOUT_US: u32 = 40_000_000;

static LINK_CONNECTED: AtomicBool = AtomicBool::new(false);

/// Returns whether the register-level Link Layer currently owns an active
/// connection. This does not infer pairing, CCCD state or HID delivery.
pub fn is_connected() -> bool {
    LINK_CONNECTED.load(Ordering::Acquire)
}

/// Legacy advertiser configuration owned by the controller.
struct Advertiser {
    address: DeviceAddress,
    interval: Duration,
    channels: [bool; 3],
    data: AdvertisingData,
    scan_response: AdvertisingData,
    enabled: bool,
}

#[derive(Clone, Copy)]
struct PendingData {
    bytes: [u8; MAX_DATA_PDU_PAYLOAD],
    len: usize,
    llid: u8,
    owner_token: u32,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum TransmittedPdu {
    None,
    Control,
    Acl,
    Empty,
}

#[derive(Clone, Copy)]
struct LocalConnectionParamRequest {
    interval_min: u16,
    interval_max: u16,
    latency: u16,
    supervision_timeout: u16,
}

impl LocalConnectionParamRequest {
    fn accepts(self, update: ConnectionUpdateInd) -> bool {
        connection_update_matches_request(
            self.interval_min,
            self.interval_max,
            self.latency,
            self.supervision_timeout,
            update,
        )
    }
}

struct ActiveConnection {
    handle: u16,

    request: ConnectInd,
    channels: ChannelSelection1,
    next_anchor: Instant,
    last_sync_anchor: Instant,
    receive_window_size_us: u64,
    event_counter: u16,
    supervision_deadline: Instant,
    transmit_sn: u8,
    expected_sn: u8,
    next_acl_owner_token: u32,
    next_control_owner_token: u32,
    pending: Option<PendingData>,
    queued_acl: [Option<PendingData>; ACL_QUEUE_CAPACITY],
    pending_control: Option<PendingData>,
    // Reserved response slot; an unacknowledged owner is never replaced.
    priority_control: Option<PendingData>,
    queued_control: Option<PendingData>,
    transmitted: TransmittedPdu,
    tx_frame: [u8; MAX_DATA_PDU_WIRE_LEN],
    tx_frame_len: usize,
    tx_frame_fresh: bool,

    pending_connection_update: Option<ConnectionUpdateInd>,
    pending_channel_map: Option<ChannelMapInd>,
    local_connection_param_request: Option<LocalConnectionParamRequest>,
    procedures: ProcedureManager,
    last_rssi: i8,
}

impl ActiveConnection {
    fn new(request: ConnectInd, connect_ind_end_us: u64) -> Result<Self, RadioError> {
        Self::new_for(CONNECTION_HANDLE, request, connect_ind_end_us)
    }

    fn new_for(
        handle: u16,
        request: ConnectInd,
        connect_ind_end_us: u64,
    ) -> Result<Self, RadioError> {
        let channels = ChannelSelection1::new(request.channel_map, request.hop_increment)?;
        let now = Instant::now();
        let first_anchor = Instant::from_micros(connect_ind_end_us)
            + Duration::from_micros((u64::from(request.window_offset) + 1) * 1_250);
        let receive_window_size_us = u64::from(request.window_size) * 1_250;

        Ok(Self {
            handle,

            request,
            channels,
            next_anchor: first_anchor,
            last_sync_anchor: Instant::from_micros(connect_ind_end_us),
            receive_window_size_us,
            event_counter: 0,
            supervision_deadline: now
                + Duration::from_micros(u64::from(request.supervision_timeout) * 10_000),
            transmit_sn: 0,
            expected_sn: 0,
            next_acl_owner_token: 1,
            next_control_owner_token: 0x8000_0001,
            pending: None,
            queued_acl: [None; ACL_QUEUE_CAPACITY],
            pending_control: None,
            priority_control: None,
            queued_control: None,
            transmitted: TransmittedPdu::None,
            tx_frame: [0; MAX_DATA_PDU_WIRE_LEN],
            tx_frame_len: 0,
            tx_frame_fresh: false,

            pending_connection_update: None,
            pending_channel_map: None,
            local_connection_param_request: None,
            procedures: ProcedureManager::new(),
            last_rssi: -127,
        })
    }

    fn interval(&self) -> Duration {
        Duration::from_micros(u64::from(self.request.interval) * 1_250)
    }

    fn next_channel(&mut self) -> Channel {
        self.channels.next_channel()
    }

    fn window_widening_us(&self) -> u64 {
        window_widening_us(
            self.next_anchor
                .saturating_duration_since(self.last_sync_anchor)
                .as_micros(),
            self.request.sleep_clock_accuracy,
            LOCAL_SLEEP_CLOCK_PPM,
        )
    }

    fn receive_open(&self) -> Instant {
        self.next_anchor
            - Duration::from_micros(self.receive_setup_lead_us() + self.window_widening_us())
    }

    fn scheduler_open(&self) -> Instant {
        self.receive_open()
    }

    fn take_receive_timeout(&mut self) -> Duration {
        let widening_us = self.window_widening_us();
        let timeout_us = self
            .receive_setup_lead_us()
            .saturating_add(widening_us.saturating_mul(2))
            .saturating_add(self.receive_window_size_us)
            .saturating_add(MAX_DATA_PACKET_AIRTIME_US);
        // WinSize belongs only to the first transmit window after CONNECT_IND
        // or LL_CONNECTION_UPDATE_IND. Future events use accumulated widening.
        self.receive_window_size_us = 0;
        Duration::from_micros(timeout_us)
    }

    fn receive_setup_lead_us(&self) -> u64 {
        RX_SETUP_LEAD_US
    }

    fn receive_close(&self) -> Instant {
        let instant_window_us = self
            .pending_connection_update
            .filter(|update| update.instant == self.event_counter)
            .map_or(self.receive_window_size_us, |update| {
                u64::from(update.window_size) * 1_250
            });
        self.next_anchor
            + Duration::from_micros(
                self.window_widening_us()
                    .saturating_add(instant_window_us)
                    .saturating_add(MAX_DATA_PACKET_AIRTIME_US),
            )
    }

    /// Advance CSA#1, instants and anchors for connection events whose entire
    /// receive window elapsed while another executor task held the CPU.
    /// Touching RF for an anchor that is already in the past makes recovery
    /// slower than the peer's event cadence and can turn one long crypto call
    /// into a supervision timeout.
    fn skip_expired_events(&mut self, now: Instant) -> Result<(u16, bool), u8> {
        let mut count = 0u16;
        let mut notify_host = false;
        while now > self.receive_close() {
            let (applied, changed) = self.apply_instants()?;
            notify_host |= changed || (applied && self.local_connection_param_request.is_some());
            if applied {
                self.local_connection_param_request = None;
            }
            let _ = self.next_channel();
            self.schedule_following_event();
            count = count.checked_add(1).ok_or(0x08)?;
        }
        Ok((count, notify_host))
    }

    fn synchronize(&mut self, anchor_us: u64) {
        let anchor = Instant::from_micros(anchor_us);
        self.next_anchor = anchor;
        self.last_sync_anchor = anchor;
        self.receive_window_size_us = 0;
    }

    fn schedule_following_event(&mut self) {
        let old_interval = self.interval();
        self.event_counter = self.event_counter.wrapping_add(1);

        self.next_anchor += old_interval;
        if let Some(update) = self.pending_connection_update
            && update.instant == self.event_counter
        {
            // The new transmit window begins one old interval plus WinOffset
            // after the preceding anchor point.
            self.next_anchor += Duration::from_micros(u64::from(update.window_offset) * 1_250);
        }
    }

    /// Returns `(connection_update_applied, parameters_changed)`.
    fn apply_instants(&mut self) -> Result<(bool, bool), u8> {
        if let Some(update) = self.pending_channel_map
            && update.instant == self.event_counter
        {
            self.channels.update_map(update.map).map_err(|_| 0x1e)?;
            self.request.channel_map = update.map;
            self.pending_channel_map = None;
        }

        let mut connection_update_applied = false;
        let mut parameters_changed = false;
        if let Some(update) = self.pending_connection_update
            && update.instant == self.event_counter
        {
            connection_update_applied = true;
            parameters_changed = self.request.interval != update.interval
                || self.request.latency != update.latency
                || self.request.supervision_timeout != update.supervision_timeout;
            self.request.window_size = update.window_size;
            self.receive_window_size_us = u64::from(update.window_size) * 1_250;
            self.request.window_offset = update.window_offset;
            self.request.interval = update.interval;
            self.request.latency = update.latency;
            self.request.supervision_timeout = update.supervision_timeout;
            self.supervision_deadline = Instant::now()
                + Duration::from_micros(u64::from(update.supervision_timeout) * 10_000);
            self.pending_connection_update = None;
        }
        self.procedures
            .on_connection_event(self.event_counter, 0x28);
        Ok((connection_update_applied, parameters_changed))
    }

    fn queue_acl(&mut self, packet: &Packet) -> Result<(), u8> {
        if packet.len < 4 {
            return Err(HCI_COMMAND_DISALLOWED);
        }
        let raw_handle = u16::from_le_bytes([packet.bytes[0], packet.bytes[1]]);
        let handle = raw_handle & 0x0fff;
        let length = usize::from(u16::from_le_bytes([packet.bytes[2], packet.bytes[3]]));
        if handle != self.handle || length + 4 != packet.len || length > MAX_DATA_PDU_PAYLOAD {
            return Err(HCI_INVALID_PARAMETERS);
        }
        let boundary = (raw_handle >> 12) & 0x03;
        let llid = if boundary == 1 {
            LLID_CONTINUATION
        } else {
            LLID_START
        };

        let mut data = PendingData {
            bytes: [0; MAX_DATA_PDU_PAYLOAD],
            len: length,
            llid,
            owner_token: self.next_acl_owner_token,
        };
        data.bytes[..length].copy_from_slice(&packet.bytes[4..4 + length]);

        let queued = if self.pending.is_none() {
            self.pending = Some(data);
            true
        } else if let Some(slot) = self.queued_acl.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(data);
            true
        } else {
            false
        };
        if !queued {
            return Err(HCI_COMMAND_DISALLOWED);
        }

        self.next_acl_owner_token = self.next_acl_owner_token.wrapping_add(1).max(1);
        Ok(())
    }

    fn allocate_control_token(&mut self) -> ProcedureToken {
        let owner_token = self.next_control_owner_token;
        self.next_control_owner_token = self.next_control_owner_token.wrapping_add(1) | 0x8000_0000;
        ProcedureToken {
            owner_token,
            generation: owner_token as u8,
        }
    }

    fn queue_control_with_token(&mut self, bytes: &[u8], token: ProcedureToken) -> bool {
        if bytes.len() > MAX_DATA_PDU_PAYLOAD {
            return false;
        }
        let mut data = PendingData {
            bytes: [0; MAX_DATA_PDU_PAYLOAD],
            len: bytes.len(),
            llid: LLID_CONTROL,
            owner_token: token.owner_token,
        };
        data.bytes[..bytes.len()].copy_from_slice(bytes);
        if self.pending_control.is_none() {
            self.pending_control = Some(data);
            true
        } else if self.queued_control.is_none() {
            self.queued_control = Some(data);
            true
        } else {
            false
        }
    }

    fn queue_control_owned(&mut self, bytes: &[u8]) -> Option<ProcedureToken> {
        let token = self.allocate_control_token();
        self.queue_control_with_token(bytes, token).then_some(token)
    }

    fn queue_control(&mut self, bytes: &[u8]) -> bool {
        self.queue_control_owned(bytes).is_some()
    }

    fn queue_remote_procedure_response(
        &mut self,
        kind: ProcedureKind,
        request_opcode: u8,
        response: &[u8],
    ) -> bool {
        self.queue_remote_procedure_response_with_followup(
            kind,
            request_opcode,
            response,
            None,
            None,
        )
    }

    fn queue_remote_procedure_response_with_followup(
        &mut self,
        kind: ProcedureKind,
        request_opcode: u8,
        response: &[u8],
        expected_response_opcode: Option<u8>,
        response_timeout_us: Option<u32>,
    ) -> bool {
        let Some(&response_opcode) = response.first() else {
            return false;
        };
        let token = self.allocate_control_token();
        let context = ProcedureContext::new(
            kind,
            ProcedureInitiator::Remote,
            token,
            response_opcode,
            expected_response_opcode,
            None,
            response_timeout_us,
        );
        match self.procedures.start_remote(context) {
            Ok(()) => {}
            Err(ProcedureError::Collision(error)) => {
                return self.queue_control(&[0x11, request_opcode, error]);
            }
            Err(_) => return self.queue_control(&[0x11, request_opcode, 0x23]),
        }
        if self
            .procedures
            .response_queued(token, response_opcode)
            .is_err()
            || !self.queue_control_with_token(response, token)
        {
            let _ = self.procedures.fail(ProcedureInitiator::Remote, 0x3a);
            return false;
        }
        true
    }

    fn remote_connection_parameters_active(&self) -> bool {
        self.procedures.remote().is_some_and(|context| {
            context.kind == ProcedureKind::ConnectionParameters
                && !matches!(
                    context.phase,
                    ProcedurePhase::Complete | ProcedurePhase::Failed
                )
        })
    }

    fn start_remote_instant_procedure(
        &mut self,
        kind: ProcedureKind,
        opcode: u8,
        instant: u16,
    ) -> Result<(), u8> {
        let token = self.allocate_control_token();
        let context = ProcedureContext::new(
            kind,
            ProcedureInitiator::Remote,
            token,
            opcode,
            None,
            Some(instant),
            None,
        );
        match self.procedures.start_remote(context) {
            Ok(()) => Ok(()),
            Err(ProcedureError::Collision(error)) => Err(error),
            Err(_) => Err(0x23),
        }
    }

    fn queue_local_procedure_request(
        &mut self,
        kind: ProcedureKind,
        request: &[u8],
        expected_response_opcode: u8,
    ) -> bool {
        self.queue_local_procedure(
            kind,
            request,
            Some(expected_response_opcode),
            Some(PROCEDURE_RESPONSE_TIMEOUT_US),
        )
    }

    fn queue_local_procedure(
        &mut self,
        kind: ProcedureKind,
        request: &[u8],
        expected_response_opcode: Option<u8>,
        response_timeout_us: Option<u32>,
    ) -> bool {
        let Some(&request_opcode) = request.first() else {
            return false;
        };
        let token = self.allocate_control_token();
        let context = ProcedureContext::new(
            kind,
            ProcedureInitiator::Local,
            token,
            request_opcode,
            expected_response_opcode,
            None,
            response_timeout_us,
        );
        if self.procedures.start_local(context).is_err() {
            return false;
        }
        if !self.queue_control_with_token(request, token) {
            let _ = self.procedures.fail(ProcedureInitiator::Local, 0x3a);
            return false;
        }
        true
    }

    fn local_procedure_active(&self, kind: ProcedureKind) -> bool {
        self.procedures.local().is_some_and(|context| {
            context.kind == kind
                && !matches!(
                    context.phase,
                    ProcedurePhase::Complete | ProcedurePhase::Failed
                )
        })
    }

    fn procedure_for_control_owner(
        &self,
        owner_token: u32,
    ) -> Option<(ProcedureInitiator, ProcedureToken)> {
        for (initiator, context) in [
            (ProcedureInitiator::Local, self.procedures.local()),
            (ProcedureInitiator::Remote, self.procedures.remote()),
        ] {
            if let Some(context) = context
                && context.token.owner_token == owner_token
            {
                return Some((initiator, context.token));
            }
        }
        None
    }

    fn plan_pending_control_procedure(&mut self) -> Result<(), ()> {
        let Some(owner_token) = self.pending_control.map(|pending| pending.owner_token) else {
            return Ok(());
        };
        let Some((initiator, token)) = self.procedure_for_control_owner(owner_token) else {
            return Ok(());
        };
        self.procedures.tx_planned(initiator, token).map_err(|_| ())
    }

    fn complete_pending_control_transmit(&mut self) -> Result<(), ()> {
        let Some(owner_token) = self.pending_control.map(|pending| pending.owner_token) else {
            return Ok(());
        };
        let Some((initiator, token)) = self.procedure_for_control_owner(owner_token) else {
            return Ok(());
        };
        self.procedures.tx_done(initiator, token).map_err(|_| ())
    }

    fn acknowledge_pending_control(&mut self, now_us: u64) -> Result<(), ()> {
        let Some(owner_token) = self.pending_control.map(|pending| pending.owner_token) else {
            return Ok(());
        };
        let Some((initiator, token)) = self.procedure_for_control_owner(owner_token) else {
            return Ok(());
        };
        self.procedures
            .nesn_ack(initiator, token, now_us)
            .map_err(|_| ())
    }

    fn promote_priority_control_if_idle(&mut self) {
        if self.transmitted == TransmittedPdu::None
            && self.tx_frame_len == 0
            && self.priority_control.is_some()
        {
            // Swap rather than overwrite: an unsent ordinary procedure is
            // paused in the priority slot and restored after the priority
            // PDU's NESN ACK.
            core::mem::swap(&mut self.pending_control, &mut self.priority_control);
        }
    }
}

impl Default for Advertiser {
    fn default() -> Self {
        Self {
            address: DeviceAddress::random_static([0x82, 0x58, 0x2a, 0x01, 0x00, 0xc0]).unwrap(),
            interval: Duration::from_millis(200),
            channels: [true; 3],
            data: AdvertisingData::new(),
            scan_response: AdvertisingData::new(),
            enabled: false,
        }
    }
}

/// Owns RF registers and translates HCI commands into Link Layer procedures.
pub struct Runner {
    radio: Radio,

    random: ControllerRng,
    advertiser: Advertiser,
}

impl Runner {
    /// Creates the unique register-level Link Layer runner.
    pub fn new(_token: crate::RadioToken, random: ChaCha20Rng) -> Result<Self, RadioError> {
        let mut radio = Radio::new(_token);
        radio.initialize()?;
        let random = controller_rng(random);
        Ok(Self {
            radio,

            random,
            advertiser: Advertiser::default(),
        })
    }

    pub async fn run(&mut self) -> ! {
        loop {
            if self.advertiser.enabled {
                match select(
                    controller::receive_host_packet(),
                    Timer::after(self.advertiser.interval),
                )
                .await
                {
                    Either::First(packet) => self.handle_host_packet(packet).await,
                    Either::Second(_) => self.advertising_event().await,
                }
                continue;
            }

            let packet = controller::receive_host_packet().await;
            self.handle_host_packet(packet).await;
        }
    }

    async fn handle_host_packet(&mut self, packet: Packet) {
        match packet.kind {
            PacketKind::Cmd => self.handle_command(&packet.bytes[..packet.len]).await,
            PacketKind::AclData => {
                // ACL is accepted only after CONNECT_IND has created a link.
                // Returning credits here would falsely advertise a data path.
            }
            _ => {}
        }
    }

    async fn advertising_event(&mut self) {
        let mut output = [0; MAX_LEGACY_ADV_PDU_LEN];
        let channels = [Channel::ADV_37, Channel::ADV_38, Channel::ADV_39];
        for (slot, channel) in channels.into_iter().enumerate() {
            if !self.advertiser.channels[slot] || !self.advertiser.enabled {
                continue;
            }
            let pdu =
                match build_adv_ind(self.advertiser.address, &self.advertiser.data, &mut output) {
                    Ok(pdu) => pdu,
                    Err(_) => return,
                };
            let mut response = [0; MAX_LEGACY_ADV_PDU_LEN];
            let scan_response = match build_scan_rsp(
                self.advertiser.address,
                &self.advertiser.scan_response,
                &mut response,
            ) {
                Ok(pdu) => pdu,
                Err(_) => return,
            };
            let mut receive = [0u8; MAX_LEGACY_ADV_PDU_LEN];
            let received = self
                .radio
                .advertising_exchange(
                    channel,
                    Duration::from_micros(500),
                    pdu,
                    scan_response,
                    self.advertiser.address.bytes,
                    &mut receive,
                )
                .await;
            let Ok(received) = received else { continue };
            if is_scan_request(received.payload, self.advertiser.address) {
                if !received.crc_ok {
                    continue;
                }
                // The response was sent by the hardware RX-to-TX leg before
                // this packet is returned to software.
                continue;
            }
            if received
                .payload
                .first()
                .is_some_and(|header| header & 0x0f == 0x05)
            {
                if !received.crc_ok {
                    continue;
                }
                let connection =
                    match parse_connect_ind_detailed(received.payload, self.advertiser.address) {
                        Ok(connection) => connection,
                        Err(_) => {
                            continue;
                        }
                    };
                if let Ok(mut connection) = ActiveConnection::new(connection, received.timestamp_us)
                {
                    self.advertiser.enabled = false;
                    self.radio
                        .set_access_address(connection.request.access_address);
                    self.radio.set_crc_init(connection.request.crc_init);
                    LINK_CONNECTED.store(true, Ordering::Release);
                    self.emit_connection_complete(&connection).await;
                    // The HCI event and the first peer ACL packet share the
                    // same controller-to-host transport, but emitting the
                    // event only wakes the Host task; it does not run it.
                    // Give the Host one executor turn to publish the new
                    // Connection/SMP state before the first connection event
                    // can deliver a peer Pairing Request. Without this yield,
                    // faster no-diagnostic builds can consume that request
                    // before Trouble has installed the connection handler.
                    for _ in 0..2 {
                        yield_now().await;
                    }
                    self.run_connection(&mut connection).await;
                    LINK_CONNECTED.store(false, Ordering::Release);
                    self.radio.set_access_address(ADV_ACCESS_ADDRESS);
                    self.radio.set_crc_init(ADV_CRC_INIT);
                }
                return;
            }
        }
    }

    /// Consume one Host packet while connected. Returns true when the command
    /// requests termination. Keeping this in one helper makes the ordinary
    /// wait and the burst pre-admission wait obey identical queue/control
    /// semantics.
    async fn process_connected_host_packet(
        &mut self,
        connection: &mut ActiveConnection,
        packet: Packet,
    ) -> bool {
        match packet.kind {
            PacketKind::AclData => {
                let _ = connection.queue_acl(&packet);
                false
            }
            PacketKind::Cmd => {
                self.handle_connected_command(connection, &packet.bytes[..packet.len])
                    .await
            }
            _ => false,
        }
    }

    #[allow(clippy::collapsible_match)]
    async fn run_connection(&mut self, connection: &mut ActiveConnection) {
        loop {
            while let Some(packet) = controller::try_receive_host_packet() {
                if self.process_connected_host_packet(connection, packet).await {
                    self.emit_disconnection(connection, 0x16).await;
                    return;
                }
            }

            let now = Instant::now();
            if connection.procedures.check_timeouts(now.as_micros(), 0x22) {
                self.emit_disconnection(connection, 0x22).await;
                return;
            }
            if now >= connection.supervision_deadline {
                self.emit_disconnection(connection, 0x08).await;
                return;
            }

            let Ok((skipped, notify_update)) = connection.skip_expired_events(now) else {
                self.emit_disconnection(connection, 0x08).await;
                return;
            };
            if notify_update {
                self.emit_connection_update_complete(connection).await;
            }
            if skipped != 0 {
                continue;
            }

            match select(
                controller::receive_host_packet(),
                Timer::at(connection.scheduler_open()),
            )
            .await
            {
                Either::First(packet) => {
                    if self.process_connected_host_packet(connection, packet).await {
                        self.emit_disconnection(connection, 0x16).await;
                        return;
                    }
                }
                Either::Second(_) => {
                    // The receive-open timer can become runnable while host work still
                    // owns the executor. Revalidate the complete receive window at the
                    // RF admission point: entering an event whose close deadline is
                    // already in the past turns a normal missed anchor into a radio
                    // timeout and delays catch-up until the next outer-loop iteration.
                    let now = Instant::now();

                    let Ok((skipped, notify_update)) = connection.skip_expired_events(now) else {
                        self.emit_disconnection(connection, 0x08).await;
                        return;
                    };
                    if notify_update {
                        self.emit_connection_update_complete(connection).await;
                    }
                    if skipped != 0 {
                        continue;
                    }

                    if let Some(reason) = self.connection_event(connection).await {
                        self.emit_disconnection(connection, reason).await;
                        return;
                    }
                    connection.schedule_following_event();
                    // A successful bounded Channel::send does not yield. Give
                    // the already-woken Host completion/writer/TX pipeline one
                    // cooperative turn after RF and after the next absolute
                    // anchor is committed. This never runs in the IFS path and
                    // keeps all application parsing outside the Link Layer.
                    yield_now().await;
                }
            }
        }
    }

    /// Returns a disconnect reason when this event terminates the connection.
    async fn connection_event(&mut self, connection: &mut ActiveConnection) -> Option<u8> {
        match connection.apply_instants() {
            Ok((applied, changed)) => {
                let notify =
                    changed || (applied && connection.local_connection_param_request.is_some());
                if applied {
                    connection.local_connection_param_request = None;
                }
                if notify {
                    self.emit_connection_update_complete(connection).await;
                }
            }
            Err(reason) => return Some(reason),
        }
        let channel = connection.next_channel();
        let mut rx = [0u8; MAX_DATA_PDU_WIRE_LEN];
        let receive_deadline = connection.receive_close();
        let timeout = connection.take_receive_timeout();
        connection.promote_priority_control_if_idle();
        if connection.transmitted == TransmittedPdu::None {
            connection.transmitted = if connection.pending_control.is_some() {
                TransmittedPdu::Control
            } else if connection.pending.is_some() {
                TransmittedPdu::Acl
            } else {
                TransmittedPdu::Empty
            };
        }
        if connection.tx_frame_len == 0 && self.build_transmit_frame(connection).is_err() {
            return Some(0x1f);
        }
        refresh_transmit_sequence_header(
            &mut connection.tx_frame[0],
            connection.transmit_sn,
            connection.expected_sn,
        );
        // RX precedes TX: the first peer packet can only ACK an owner sent in
        // an earlier event. A frame staged for this event is not yet on air.
        let outstanding = !connection.tx_frame_fresh;
        let exchange = self
            .radio
            .exchange(
                channel,
                receive_deadline,
                timeout,
                &connection.tx_frame[..connection.tx_frame_len],
                &mut rx,
            )
            .await
            .ok()?;
        if connection.tx_frame_fresh && exchange.transmitted {
            if connection.transmitted == TransmittedPdu::Control
                && connection.complete_pending_control_transmit().is_err()
            {
                return Some(0x1f);
            }
            connection.tx_frame_fresh = false;
        }
        let sent_terminate = exchange.transmitted
            && connection.transmitted == TransmittedPdu::Control
            && connection.local_procedure_active(ProcedureKind::Termination)
            && connection
                .pending_control
                .as_ref()
                .is_some_and(|pdu| pdu.bytes[0] == 0x02);
        let received = &exchange.received;
        if !received.crc_ok {
            return None;
        }
        let (&header, remainder) = received.payload.split_first()?;
        let (&length, body) = remainder.split_first()?;
        if usize::from(length) > MAX_DATA_PDU_PAYLOAD || header & 0x03 == 0 {
            return None;
        }
        let payload = body.get(..usize::from(length))?;
        connection.last_rssi = received.rssi_dbm;
        connection.supervision_deadline = Instant::now()
            + Duration::from_micros(u64::from(connection.request.supervision_timeout) * 10_000);
        let anchor = received
            .timestamp_us
            .saturating_sub(((1 + 4 + 2 + usize::from(length) + 3) * 8) as u64);
        if acknowledges_transmit_owner(header, connection.transmit_sn, outstanding, false) {
            if connection.transmitted == TransmittedPdu::Control
                && connection
                    .acknowledge_pending_control(Instant::now().as_micros())
                    .is_err()
            {
                return Some(0x1f);
            }
            match connection.transmitted {
                TransmittedPdu::Control => {
                    connection.pending_control = connection
                        .priority_control
                        .take()
                        .or_else(|| connection.queued_control.take());
                }
                TransmittedPdu::Acl => {
                    connection.pending = connection.queued_acl[0].take();
                    for index in 0..connection.queued_acl.len() - 1 {
                        connection.queued_acl[index] = connection.queued_acl[index + 1].take();
                    }
                    controller::completed_acl(connection.handle, 1).await;
                }
                TransmittedPdu::Empty | TransmittedPdu::None => {}
            }
            connection.transmitted = TransmittedPdu::None;
            connection.tx_frame_len = 0;
            connection.tx_frame_fresh = false;
            connection.transmit_sn ^= 1;
        }
        let mut terminated = None;
        if (header >> 3) & 1 == connection.expected_sn {
            connection.expected_sn ^= 1;
            match header & 0x03 {
                LLID_CONTINUATION | LLID_START if !payload.is_empty() => {
                    let flags = if header & 0x03 == LLID_CONTINUATION {
                        HCI_PB_CONTINUING
                    } else {
                        HCI_PB_FIRST_FLUSHABLE
                    };
                    let _ = controller::send_acl(connection.handle | flags, payload).await;
                }
                LLID_CONTROL => terminated = self.handle_ll_control(connection, payload).await,
                _ => {}
            }
        }
        match connection.apply_instants() {
            Ok((applied, changed)) => {
                let notify =
                    changed || (applied && connection.local_connection_param_request.is_some());
                if applied {
                    connection.local_connection_param_request = None;
                }
                if notify {
                    self.emit_connection_update_complete(connection).await;
                }
            }
            Err(reason) => return Some(reason),
        }
        connection.synchronize(anchor);
        if sent_terminate {
            return Some(0x16);
        }
        terminated
    }

    fn build_transmit_frame(&mut self, connection: &mut ActiveConnection) -> Result<(), ()> {
        if connection.transmitted == TransmittedPdu::Control {
            connection.plan_pending_control_procedure()?;
        }
        let selected = match connection.transmitted {
            TransmittedPdu::Control => connection.pending_control.as_ref(),
            TransmittedPdu::Acl => connection.pending.as_ref(),
            TransmittedPdu::Empty | TransmittedPdu::None => None,
        };
        let (llid, payload) = match selected {
            Some(data) => (data.llid, &data.bytes[..data.len]),
            None => (LLID_CONTINUATION, &[][..]),
        };
        connection.tx_frame[0] =
            llid | (connection.expected_sn << 2) | (connection.transmit_sn << 3);
        connection.tx_frame[1] = payload.len() as u8;
        connection.tx_frame[2..2 + payload.len()].copy_from_slice(payload);
        connection.tx_frame_len = payload.len() + 2;
        connection.tx_frame_fresh = true;
        Ok(())
    }

    async fn handle_ll_control(
        &mut self,
        connection: &mut ActiveConnection,
        payload: &[u8],
    ) -> Option<u8> {
        let opcode = payload.first().copied()?;
        match opcode {
            0x00 => match ConnectionUpdateInd::parse(payload, connection.event_counter) {
                Ok(update) if connection.pending_connection_update.is_none() => {
                    if let Some(request) = connection.local_connection_param_request
                        && !request.accepts(update)
                    {
                        return Some(0x20);
                    }
                    let owner_result: Result<(), u8> =
                        if connection.local_connection_param_request.is_some() {
                            connection
                                .procedures
                                .expected_response_with_instant(
                                    ProcedureInitiator::Local,
                                    0x00,
                                    update.instant,
                                )
                                .map_err(|_| 0x3d)
                        } else if connection.remote_connection_parameters_active() {
                            connection
                                .procedures
                                .expected_response_with_instant(
                                    ProcedureInitiator::Remote,
                                    0x00,
                                    update.instant,
                                )
                                .map_err(|_| 0x3d)
                        } else {
                            connection.start_remote_instant_procedure(
                                ProcedureKind::ConnectionUpdate,
                                0x00,
                                update.instant,
                            )
                        };
                    if let Err(error) = owner_result {
                        return Some(error);
                    }
                    connection.pending_connection_update = Some(update);
                }
                Ok(_) | Err(ControlPduError::Malformed | ControlPduError::InvalidParameters) => {
                    return Some(0x1e);
                }
                Err(ControlPduError::InstantPassed) => return Some(0x28),
            },
            0x01 => match ChannelMapInd::parse(payload, connection.event_counter) {
                Ok(update) if connection.pending_channel_map.is_none() => {
                    if connection.local_connection_param_request.is_some() {
                        connection.local_connection_param_request = None;
                        let collision = procedure_collision_error(
                            InstantProcedure::ConnectionParametersRequest,
                            InstantProcedure::ChannelMapUpdate,
                        );
                        let _ = connection
                            .procedures
                            .fail(ProcedureInitiator::Local, collision);
                        self.emit_connection_update_result(connection, collision)
                            .await;
                    }
                    if let Err(error) = connection.start_remote_instant_procedure(
                        ProcedureKind::ChannelMapUpdate,
                        0x01,
                        update.instant,
                    ) {
                        return Some(error);
                    }
                    connection.pending_channel_map = Some(update);
                }
                Ok(_) | Err(ControlPduError::Malformed | ControlPduError::InvalidParameters) => {
                    return Some(0x1e);
                }
                Err(ControlPduError::InstantPassed) => return Some(0x28),
            },
            0x02 if payload.len() == 2 => {
                return Some(payload[1]);
            }
            0x02 => return Some(0x1e),
            0x07 if payload.len() == 2 => {
                if payload[1] == 0x0f && connection.local_connection_param_request.is_some() {
                    connection.local_connection_param_request = None;
                    let _ = connection.procedures.fail(ProcedureInitiator::Local, 0x1a);
                    self.emit_connection_update_result(connection, 0x1a).await;
                }
            }
            0x11 if payload.len() == 3 => {
                if payload[1] == 0x0f && connection.local_connection_param_request.is_some() {
                    connection.local_connection_param_request = None;
                    let _ = connection
                        .procedures
                        .fail(ProcedureInitiator::Local, payload[2]);
                    self.emit_connection_update_result(connection, payload[2])
                        .await;
                } else if payload[1] == 0x10 && connection.remote_connection_parameters_active() {
                    let _ = connection
                        .procedures
                        .fail(ProcedureInitiator::Remote, payload[2]);
                    self.emit_connection_update_result(connection, payload[2])
                        .await;
                }
            }
            // Connection Parameters Request, and Extended
            // Reject Indication. The response is the intersection with the
            // initiator's feature set, as required for LL_FEATURE_RSP.
            0x08 if payload.len() == 9 => {
                let response = [0x09, payload[1] & 0x06, 0, 0, 0, 0, 0, 0, 0];
                let _ = connection.queue_remote_procedure_response(
                    ProcedureKind::FeatureExchange,
                    0x08,
                    &response,
                );
            }
            0x0c if payload.len() == 6 => {
                let _ = connection.queue_remote_procedure_response(
                    ProcedureKind::VersionExchange,
                    0x0c,
                    &[0x0c, 0x09, 0xff, 0xff, 0x00, 0x00],
                );
            }
            0x0f => match ConnectionParamPdu::parse_request(payload, connection.event_counter) {
                Ok(request) => {
                    let collision = if connection.pending_channel_map.is_some() {
                        Some(procedure_collision_error(
                            InstantProcedure::ChannelMapUpdate,
                            InstantProcedure::ConnectionParametersRequest,
                        ))
                    } else if connection.pending_connection_update.is_some() {
                        Some(procedure_collision_error(
                            InstantProcedure::ConnectionUpdate,
                            InstantProcedure::ConnectionParametersRequest,
                        ))
                    } else {
                        None
                    };
                    if let Some(error) = collision {
                        let _ = connection.queue_control(&[0x11, 0x0f, error]);
                    } else {
                        if connection.local_connection_param_request.is_some() {
                            connection.local_connection_param_request = None;
                            let collision = procedure_collision_error(
                                InstantProcedure::ConnectionParametersRequest,
                                InstantProcedure::ConnectionParametersRequest,
                            );
                            let _ = connection
                                .procedures
                                .fail(ProcedureInitiator::Local, collision);
                            self.emit_connection_update_result(connection, collision)
                                .await;
                        }
                        // The Core permits an acceptable request to be handled
                        // autonomously. Echoing the validated parameter set
                        // avoids a Host round-trip and starts the Peripheral's
                        // response leg of the procedure.
                        let mut response = [0u8; ConnectionParamPdu::PDU_LEN];
                        request.write(ConnectionParamPdu::RESPONSE_OPCODE, &mut response);
                        let _ = connection.queue_remote_procedure_response_with_followup(
                            ProcedureKind::ConnectionParameters,
                            0x0f,
                            &response,
                            Some(0x00),
                            Some(PROCEDURE_RESPONSE_TIMEOUT_US),
                        );
                    }
                }
                Err(ControlPduError::Malformed | ControlPduError::InvalidParameters) => {
                    let _ = connection.queue_control(&[0x11, 0x0f, 0x1e]);
                }
                Err(ControlPduError::InstantPassed) => unreachable!(),
            },
            // A Central shall not send LL_CONNECTION_PARAM_RSP.
            0x10 => {
                let _ = connection.queue_control(&[0x11, 0x10, 0x1e]);
            }
            0x12 if payload.len() == 1 => {
                let _ =
                    connection.queue_remote_procedure_response(ProcedureKind::Ping, 0x12, &[0x13]);
            }
            unsupported => {
                let _ = connection.queue_control(&[0x07, unsupported]);
            }
        }
        None
    }
}

pub use ble_controller::link_layer::{Connection, ConnectionParameters, State};

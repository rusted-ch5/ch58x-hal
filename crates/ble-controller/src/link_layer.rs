//! Link Layer resource objects and event state.

use crate::Channel;

pub const ADV_ACCESS_ADDRESS: u32 = 0x8e89_bed6;
pub const ADV_CRC_INIT: u32 = 0x55_55_55;

/// Data Physical Channel header bit carrying the next expected sequence
/// number. The Link Layer authenticates neither NESN, SN nor MD in CCM AAD,
/// so a hardware backend may patch these three bits at its RX-to-TX service
/// point without re-encrypting the payload.
pub const DATA_HEADER_NESN: u8 = 1 << 2;
/// Data Physical Channel sequence-number bit.
pub const DATA_HEADER_SN: u8 = 1 << 3;
/// Data Physical Channel More Data bit.
pub const DATA_HEADER_MD: u8 = 1 << 4;
pub const DATA_HEADER_SEQUENCE_MASK: u8 = DATA_HEADER_NESN | DATA_HEADER_SN;

/// Return whether the peer header acknowledges a PDU which was already
/// outstanding when the connection event began.
///
/// A Peripheral receives the Central packet before transmitting its response.
/// That packet can acknowledge a retry from an earlier event, but it cannot
/// acknowledge a fresh response which has not gone on air yet.
pub const fn acknowledges_outstanding_transmit(
    peer_header: u8,
    transmit_sn: u8,
    outstanding_at_event_entry: bool,
) -> bool {
    outstanding_at_event_entry && ((peer_header >> 2) & 1) != transmit_sn
}

/// Return whether this exact peer packet may acknowledge the selected local
/// transmit owner.
///
/// A packet received before the Peripheral response may acknowledge only an
/// owner carried into the event. A packet captured by an explicit follow-RX
/// after that response may also acknowledge the freshly transmitted owner.
pub const fn acknowledges_transmit_owner(
    peer_header: u8,
    transmit_sn: u8,
    outstanding_at_event_entry: bool,
    received_after_primary_transmit: bool,
) -> bool {
    (outstanding_at_event_entry || received_after_primary_transmit)
        && ((peer_header >> 2) & 1) != transmit_sn
}

/// Protocol-owned sequence state for one ACL connection.
///
/// This small value is intentionally independent of the CH582 backend. It is
/// also used by the bounded connection-event burst path, where the radio must
/// choose a retransmission or the next staged PDU inside the 150-us IFS.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DataSequenceState {
    pub transmit_sn: u8,
    pub expected_sn: u8,
}

/// Result of observing one valid peer Data Physical Channel header.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DataSequenceObservation {
    /// The peer advanced NESN and therefore acknowledged our outstanding PDU.
    pub acknowledged_transmit: bool,
    /// The peer used the sequence number we expected, so its payload is new.
    pub accepted_receive: bool,
    /// The peer asks the Central to keep the current connection event open.
    pub peer_more_data: bool,
}

/// Which of the two physical DMA slots a Peripheral connection-event service
/// point must transmit next.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConnectionEventTxSlot {
    Slot0,
    Slot1,
}

/// Stable identity of one logical payload while it occupies a physical slot.
/// The generation changes every time that slot is reused.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionEventTxToken {
    pub payload_id: u8,
    /// Controller-owned identity which remains stable while the same Host ACL
    /// payload moves between queue, DMA slot, retry and ACK completion.
    pub owner_token: u32,
    pub generation: u8,
}

pub const MAX_CONNECTION_EVENT_TX_PAYLOADS: usize = 8;

/// One physical-slot refill which becomes safe after the selected response has
/// completed transmission.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionEventTxRefill {
    pub slot: ConnectionEventTxSlot,
    pub token: ConnectionEventTxToken,
}

/// Complete ownership decision for one Peripheral response service point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionEventTxDecision {
    pub response_slot: ConnectionEventTxSlot,
    /// `None` denotes a zero-length Data PDU. It still participates in the
    /// SN/NESN protocol but never completes a Host ACL buffer.
    pub response_token: Option<ConnectionEventTxToken>,
    /// The selected token has already completed one physical transmission and
    /// remains outstanding only because the peer did not advance NESN.
    ///
    /// Encrypted backends use this distinction to retain the original
    /// ciphertext and packet counter. A retry must never consume another CCM
    /// counter merely because it reached TX_DONE again.
    pub response_is_retry: bool,
    pub acknowledged_response: bool,
    pub acknowledged_token: Option<ConnectionEventTxToken>,
    pub refill_after_transmit: Option<ConnectionEventTxRefill>,
    /// At least one distinct logical payload is guaranteed staged after this
    /// response. This, not merely a non-empty current response, drives MD.
    pub more_payloads_after_response: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConnectionEventTxError {
    TooManyPayloads,
    AckWithoutOutstandingResponse,
    ResponsePlanAlreadyPending,
    CompletionDoesNotMatchDecision,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ConnectionEventOutstanding {
    slot: ConnectionEventTxSlot,
    token: Option<ConnectionEventTxToken>,
}

/// Portable rolling ownership policy for a hardware-chained connection event.
///
/// Two physical DMA slots carry an arbitrary bounded sequence of logical
/// payload IDs. The active slot remains outstanding across retries until NESN
/// acknowledges its exact payload. Once ACKed, the other pre-staged slot is
/// selected for the current response and the released slot may be refilled
/// after that response completes. This keeps buffer reuse out of the 150-us
/// RX-to-TX service point while allowing 2/4/8-PDU event tests to share the
/// same protocol state machine.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionEventTxState {
    slots: [Option<ConnectionEventTxToken>; 2],
    generations: [u8; 2],
    outstanding: Option<ConnectionEventOutstanding>,
    planned: Option<ConnectionEventTxDecision>,
    payload_count: u8,
    next_payload_id: u8,
    owner_tokens: [u32; MAX_CONNECTION_EVENT_TX_PAYLOADS],
    last_released_slot: ConnectionEventTxSlot,
}

impl ConnectionEventTxState {
    pub fn new(payload_count: u8) -> Self {
        let mut owner_tokens = [0u32; MAX_CONNECTION_EVENT_TX_PAYLOADS];
        let bounded_count = payload_count.min(MAX_CONNECTION_EVENT_TX_PAYLOADS as u8);
        for payload_id in 0..bounded_count {
            owner_tokens[usize::from(payload_id)] = u32::from(payload_id);
        }
        Self::from_owner_tokens(owner_tokens, bounded_count)
    }

    pub fn new_with_owner_tokens(owner_tokens: &[u32]) -> Result<Self, ConnectionEventTxError> {
        if owner_tokens.len() > MAX_CONNECTION_EVENT_TX_PAYLOADS {
            return Err(ConnectionEventTxError::TooManyPayloads);
        }
        let mut copied = [0u32; MAX_CONNECTION_EVENT_TX_PAYLOADS];
        copied[..owner_tokens.len()].copy_from_slice(owner_tokens);
        Ok(Self::from_owner_tokens(copied, owner_tokens.len() as u8))
    }

    /// Resume an outstanding Empty PDU while staging new Host payloads.
    /// Slot 0 remains owned by the Empty until ACK. Only payload 0 fits in
    /// slot 1; further payloads are refilled after a real TX completion.
    /// An Empty is not a synthetic Host token and never completes a Host ACL.
    pub fn with_empty_retry(owner_tokens: &[u32]) -> Result<Self, ConnectionEventTxError> {
        let mut state = Self::new_with_owner_tokens(owner_tokens)?;
        state.slots = [None, state.slots[0]];
        state.next_payload_id = u8::from(!owner_tokens.is_empty());
        state.outstanding = Some(ConnectionEventOutstanding {
            slot: ConnectionEventTxSlot::Slot0,
            token: None,
        });
        Ok(state)
    }

    fn from_owner_tokens(
        owner_tokens: [u32; MAX_CONNECTION_EVENT_TX_PAYLOADS],
        payload_count: u8,
    ) -> Self {
        let slots = match payload_count {
            0 => [None, None],
            1 => [
                Some(ConnectionEventTxToken {
                    payload_id: 0,
                    owner_token: owner_tokens[0],
                    generation: 0,
                }),
                None,
            ],
            _ => [
                Some(ConnectionEventTxToken {
                    payload_id: 0,
                    owner_token: owner_tokens[0],
                    generation: 0,
                }),
                Some(ConnectionEventTxToken {
                    payload_id: 1,
                    owner_token: owner_tokens[1],
                    generation: 0,
                }),
            ],
        };
        Self {
            slots,
            generations: [0, 0],
            outstanding: None,
            planned: None,
            payload_count,
            next_payload_id: if payload_count > 1 { 2 } else { payload_count },
            owner_tokens,
            last_released_slot: ConnectionEventTxSlot::Slot0,
        }
    }

    pub const fn slot_index(slot: ConnectionEventTxSlot) -> usize {
        match slot {
            ConnectionEventTxSlot::Slot0 => 0,
            ConnectionEventTxSlot::Slot1 => 1,
        }
    }

    const fn staged_slot(
        slots: [Option<ConnectionEventTxToken>; 2],
    ) -> Option<ConnectionEventTxSlot> {
        match (slots[0], slots[1]) {
            (Some(left), Some(right)) if right.payload_id < left.payload_id => {
                Some(ConnectionEventTxSlot::Slot1)
            }
            (Some(_), _) => Some(ConnectionEventTxSlot::Slot0),
            (None, Some(_)) => Some(ConnectionEventTxSlot::Slot1),
            (None, None) => None,
        }
    }

    /// Observe the peer NESN and choose the exact response ownership.
    ///
    /// The caller must invoke [`Self::complete_transmit`] only after hardware
    /// reports TX completion. A later peer ACK can therefore never release a
    /// staged-but-not-transmitted payload.
    pub fn select(
        &mut self,
        acknowledged_transmit: bool,
    ) -> Result<ConnectionEventTxDecision, ConnectionEventTxError> {
        let decision = self.preview_select(acknowledged_transmit)?;
        if decision.acknowledged_response {
            let outstanding = self
                .outstanding
                .take()
                .ok_or(ConnectionEventTxError::AckWithoutOutstandingResponse)?;
            self.slots[Self::slot_index(outstanding.slot)] = None;
            self.last_released_slot = outstanding.slot;
        }
        self.planned = Some(decision);
        Ok(decision)
    }

    /// Preview the same selection without cloning the ownership table or
    /// mutating any owner/generation. RF backends may prepare both NESN cases
    /// before RX, then call `select` for the actually observed valid packet.
    /// This does not grant TX ownership or authorize DMA buffer reuse.
    pub fn preview_select(
        &self,
        acknowledged_transmit: bool,
    ) -> Result<ConnectionEventTxDecision, ConnectionEventTxError> {
        if self.planned.is_some() {
            return Err(ConnectionEventTxError::ResponsePlanAlreadyPending);
        }
        let mut acknowledged_response = false;
        let mut acknowledged_token = None;
        let mut released_slot = None;
        let mut slots = self.slots;
        let mut last_released_slot = self.last_released_slot;

        if let Some(outstanding) = self.outstanding {
            if !acknowledged_transmit {
                let decision = ConnectionEventTxDecision {
                    response_slot: outstanding.slot,
                    response_token: outstanding.token,
                    response_is_retry: true,
                    acknowledged_response: false,
                    acknowledged_token: None,
                    refill_after_transmit: None,
                    more_payloads_after_response: self.slots.iter().enumerate().any(
                        |(index, token)| {
                            index != Self::slot_index(outstanding.slot) && token.is_some()
                        },
                    ) || self.next_payload_id < self.payload_count,
                };
                return Ok(decision);
            }
            acknowledged_response = true;
            acknowledged_token = outstanding.token;
            released_slot = Some(outstanding.slot);
            slots[Self::slot_index(outstanding.slot)] = None;
            last_released_slot = outstanding.slot;
        } else if acknowledged_transmit {
            return Err(ConnectionEventTxError::AckWithoutOutstandingResponse);
        }

        let response_slot = match Self::staged_slot(slots) {
            Some(slot) => slot,
            None => last_released_slot,
        };
        let response_token = slots[Self::slot_index(response_slot)];
        let mut refill_after_transmit = None;
        let released_slot = released_slot.or_else(|| {
            let candidate = match response_slot {
                ConnectionEventTxSlot::Slot0 => ConnectionEventTxSlot::Slot1,
                ConnectionEventTxSlot::Slot1 => ConnectionEventTxSlot::Slot0,
            };
            slots[Self::slot_index(candidate)]
                .is_none()
                .then_some(candidate)
        });
        if let Some(slot) = released_slot
            && self.next_payload_id < self.payload_count
        {
            let index = Self::slot_index(slot);
            let generation = self.generations[index].wrapping_add(1);
            refill_after_transmit = Some(ConnectionEventTxRefill {
                slot,
                token: ConnectionEventTxToken {
                    payload_id: self.next_payload_id,
                    owner_token: self.owner_tokens[usize::from(self.next_payload_id)],
                    generation,
                },
            });
        }

        let response_index = Self::slot_index(response_slot);
        let more_payloads_after_response = slots
            .iter()
            .enumerate()
            .any(|(index, token)| index != response_index && token.is_some())
            || refill_after_transmit.is_some()
            || self.next_payload_id < self.payload_count;

        let decision = ConnectionEventTxDecision {
            response_slot,
            response_token,
            response_is_retry: false,
            acknowledged_response,
            acknowledged_token,
            refill_after_transmit,
            more_payloads_after_response,
        };
        Ok(decision)
    }

    /// Commit one response only after the hardware TX-complete edge.
    pub fn complete_transmit(
        &mut self,
        decision: ConnectionEventTxDecision,
    ) -> Result<(), ConnectionEventTxError> {
        if self.planned != Some(decision) {
            return Err(ConnectionEventTxError::CompletionDoesNotMatchDecision);
        }
        if let Some(outstanding) = self.outstanding
            && (outstanding.slot != decision.response_slot
                || outstanding.token != decision.response_token)
        {
            return Err(ConnectionEventTxError::CompletionDoesNotMatchDecision);
        }
        if let Some(refill) = decision.refill_after_transmit {
            let index = Self::slot_index(refill.slot);
            if self.slots[index].is_some()
                || refill.token.payload_id != self.next_payload_id
                || refill.token.generation != self.generations[index].wrapping_add(1)
                || self
                    .owner_tokens
                    .get(usize::from(refill.token.payload_id))
                    .copied()
                    != Some(refill.token.owner_token)
            {
                return Err(ConnectionEventTxError::CompletionDoesNotMatchDecision);
            }
        }

        self.planned = None;
        self.outstanding = Some(ConnectionEventOutstanding {
            slot: decision.response_slot,
            token: decision.response_token,
        });
        if let Some(refill) = decision.refill_after_transmit {
            let index = Self::slot_index(refill.slot);
            self.generations[index] = refill.token.generation;
            self.slots[index] = Some(refill.token);
            self.next_payload_id += 1;
        }
        Ok(())
    }

    /// Close an event after a failed response without allowing a late TX_DONE
    /// callback to commit the stale slot generation.
    pub fn abort_transmit(
        &mut self,
        decision: ConnectionEventTxDecision,
    ) -> Result<(), ConnectionEventTxError> {
        if self.planned != Some(decision) {
            return Err(ConnectionEventTxError::CompletionDoesNotMatchDecision);
        }
        self.planned = None;
        Ok(())
    }
}

impl Default for ConnectionEventTxState {
    fn default() -> Self {
        Self::new(0)
    }
}

impl DataSequenceState {
    pub const fn new(transmit_sn: u8, expected_sn: u8) -> Self {
        Self {
            transmit_sn: transmit_sn & 1,
            expected_sn: expected_sn & 1,
        }
    }

    /// Commit the peer's NESN/SN fields in Link Layer order.
    ///
    /// CRC and, when enabled, MIC validation remain the caller's
    /// responsibility. Retransmitted peer payloads are not accepted, while an
    /// acknowledged local PDU advances the sequence used by the next staged
    /// response.
    pub const fn observe(&mut self, peer_header: u8) -> DataSequenceObservation {
        let peer_nesn = (peer_header >> 2) & 1;
        let peer_sn = (peer_header >> 3) & 1;
        let acknowledged_transmit = peer_nesn != self.transmit_sn;
        let accepted_receive = peer_sn == self.expected_sn;
        if acknowledged_transmit {
            self.transmit_sn ^= 1;
        }
        if accepted_receive {
            self.expected_sn ^= 1;
        }
        DataSequenceObservation {
            acknowledged_transmit,
            accepted_receive,
            peer_more_data: peer_header & DATA_HEADER_MD != 0,
        }
    }

    /// Patch only the sequence/MD fields of a staged Data Channel PDU.
    pub const fn response_header(self, staged_header: u8, more_data: bool) -> u8 {
        let mut header = (staged_header & !(DATA_HEADER_SEQUENCE_MASK | DATA_HEADER_MD))
            | (self.expected_sn << 2)
            | (self.transmit_sn << 3);
        if more_data {
            header |= DATA_HEADER_MD;
        }
        header
    }
}

/// Whether the Peripheral must remain ready for another Central packet.
///
/// The Central is allowed to continue when either participant advertises more
/// data. A backend may still impose a documented airtime bound, but it must
/// clear its own MD bit in the last response rather than accidentally
/// promising an unimplemented continuation.
pub const fn connection_event_may_continue(local_more_data: bool, peer_more_data: bool) -> bool {
    local_more_data || peer_more_data
}

/// Why a Peripheral stopped exchanging packets in one connection event.
///
/// This value belongs to the portable Link Layer so every radio backend and
/// diagnostic trace uses the same closure semantics. `Deadline` is a normal
/// event boundary, not a link loss; supervision is evaluated separately.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConnectionEventCloseReason {
    Continue = 0,
    BothMdClear = 1,
    Deadline = 2,
    RadioTimeout = 3,
    InvalidCrc = 4,
    InvalidMic = 5,
    ResourceLimit = 6,
    ProtocolError = 7,
}

/// One backend-independent connection-event trace record.
///
/// Hardware captures the status and timestamp at its RX/TX service point, but
/// persistence is deliberately deferred until the event has closed. This
/// keeps flash/debug formatting and atomic ring writes out of the 150-us IFS
/// path while preserving the values observed before status acknowledgement.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionEventTraceRecord {
    pub event_counter: u16,
    pub channel: u8,
    pub leg_index: u8,
    pub timestamp_us: u64,
    pub lle_status: u32,
    pub bb_status: u32,
    pub peer_header: u8,
    pub response_header: u8,
    pub received_len: u8,
    pub response_slot: u8,
    pub crc_ok: bool,
    pub acknowledged_transmit: bool,
    pub accepted_receive: bool,
    pub peer_more_data: bool,
    pub response_transmitted: bool,
    pub close_reason: ConnectionEventCloseReason,
}

impl ConnectionEventTraceRecord {
    /// Encode the stable seven-word payload used by a backend trace ring.
    ///
    /// A ring may prepend its own sequence/commit word. The close reason is
    /// `Continue` on intermediate legs and the actual event close reason on
    /// the final record.
    pub const fn encode(self) -> [u32; 7] {
        let flags = self.crc_ok as u32
            | ((self.acknowledged_transmit as u32) << 1)
            | ((self.accepted_receive as u32) << 2)
            | ((self.peer_more_data as u32) << 3)
            | ((self.response_transmitted as u32) << 4)
            | ((self.close_reason as u32) << 8);
        [
            self.event_counter as u32
                | ((self.channel as u32) << 16)
                | ((self.leg_index as u32) << 24),
            self.timestamp_us as u32,
            (self.timestamp_us >> 32) as u32,
            self.lle_status,
            self.bb_status,
            self.peer_header as u32
                | ((self.response_header as u32) << 8)
                | ((self.received_len as u32) << 16)
                | ((self.response_slot as u32) << 24),
            flags,
        ]
    }
}

/// Select the overwrite slot for a monotonically increasing trace sequence.
pub const fn connection_event_trace_slot(sequence: u32, capacity: usize) -> usize {
    assert!(
        capacity != 0,
        "connection-event trace capacity must be non-zero"
    );
    sequence as usize % capacity
}

/// Commit a service-point header only when that response reached the air.
///
/// Backends use a zero header in trace metadata when CRC or hardware status
/// prevents the response leg. That diagnostic placeholder must never replace
/// the cached, still-unacknowledged PDU: doing so would turn the next retry
/// into the reserved LLID value zero.
pub const fn committed_response_header(
    staged_header: u8,
    response_header: u8,
    response_transmitted: bool,
) -> u8 {
    if response_transmitted {
        response_header
    } else {
        staged_header
    }
}

/// Inputs known after one Central PDU and its same-IFS response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionEventDecisionInput {
    pub crc_ok: bool,
    pub mic_ok: bool,
    pub local_more_data: bool,
    pub peer_more_data: bool,
    pub before_deadline: bool,
    pub resources_available: bool,
}

/// Decide the MD bit for a queued response while preserving one bounded ACK
/// opportunity for the last fresh owner.
///
/// `require_final_ack` is a scheduling policy, not a synthetic payload: once
/// the owner becomes a retry, or the response is already Empty, it no longer
/// keeps the event open. This prevents an absent ACK from creating an unbounded
/// same-event loop.
pub const fn response_advertises_more_data(
    more_payloads_after_response: bool,
    require_final_ack: bool,
    response_has_owner: bool,
    response_is_retry: bool,
    resources_available: bool,
) -> bool {
    resources_available
        && (more_payloads_after_response
            || (require_final_ack && response_has_owner && !response_is_retry))
}

/// Portable decision for continuing or closing a connection event.
///
/// CRC/MIC failures never commit SN/NESN and therefore close the current
/// event without classifying either participant's MD bit as actionable.
pub const fn connection_event_decision(
    input: ConnectionEventDecisionInput,
) -> ConnectionEventCloseReason {
    if !input.crc_ok {
        return ConnectionEventCloseReason::InvalidCrc;
    }
    if !input.mic_ok {
        return ConnectionEventCloseReason::InvalidMic;
    }
    if !input.before_deadline {
        return ConnectionEventCloseReason::Deadline;
    }
    if !connection_event_may_continue(input.local_more_data, input.peer_more_data) {
        return ConnectionEventCloseReason::BothMdClear;
    }
    if !input.resources_available {
        return ConnectionEventCloseReason::ResourceLimit;
    }
    ConnectionEventCloseReason::Continue
}

/// Sleep-clock accuracy bins used by CONNECT_IND, in parts per million.
/// Values are the upper bound of each Core-defined SCA range.
pub const fn sleep_clock_accuracy_ppm(sca: u8) -> u16 {
    match sca & 0x07 {
        0 => 500,
        1 => 250,
        2 => 150,
        3 => 100,
        4 => 75,
        5 => 50,
        6 => 30,
        _ => 20,
    }
}

/// Accumulated Peripheral receive-window widening in microseconds.
///
/// The result is rounded up so integer arithmetic never underestimates the
/// combined clock drift. `elapsed_us` is measured since the last successful
/// synchronization point, not since connection creation.
pub const fn window_widening_us(elapsed_us: u64, central_sca: u8, local_sca_ppm: u16) -> u64 {
    let total_ppm = sleep_clock_accuracy_ppm(central_sca) as u64 + local_sca_ppm as u64;
    let whole = elapsed_us / 1_000_000;
    let remainder = elapsed_us % 1_000_000;
    whole
        .saturating_mul(total_ppm)
        .saturating_add(remainder.saturating_mul(total_ppm).div_ceil(1_000_000))
}

const fn transition_count(value: u32, bits: u8) -> u32 {
    let mask = if bits == 32 {
        u32::MAX >> 1
    } else {
        (1u32 << (bits - 1)) - 1
    };
    ((value ^ (value >> 1)) & mask).count_ones()
}

/// Validate a connection/CIS/periodic-train Access Address.
///
/// `coded_phy` enables the two additional LE Coded PHY constraints. The CH582
/// backend currently advertises only LE 1M and calls this with `false`.
pub const fn data_access_address_is_valid(access_address: u32, coded_phy: bool) -> bool {
    if access_address == ADV_ACCESS_ADDRESS
        || (access_address ^ ADV_ACCESS_ADDRESS).count_ones() == 1
    {
        return false;
    }

    let octets = access_address.to_le_bytes();
    if octets[0] == octets[1] && octets[1] == octets[2] && octets[2] == octets[3] {
        return false;
    }

    let mut previous = access_address & 1;
    let mut run = 1u8;
    let mut bit = 1u8;
    while bit < 32 {
        let current = (access_address >> bit) & 1;
        if current == previous {
            run += 1;
            if run > 6 {
                return false;
            }
        } else {
            previous = current;
            run = 1;
        }
        bit += 1;
    }

    if transition_count(access_address, 32) > 24 || transition_count(access_address >> 26, 6) < 2 {
        return false;
    }
    if coded_phy
        && ((access_address & 0xff).count_ones() < 3
            || transition_count(access_address & 0xffff, 16) > 11)
    {
        return false;
    }
    true
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum State {
    Standby,
    Advertising,
    Scanning,
    Initiating,
    Connected,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdvertisingConfig {
    pub interval_us: u32,
    pub channels: [Option<Channel>; 3],
    pub connectable: bool,
    pub scannable: bool,
}

impl Default for AdvertisingConfig {
    fn default() -> Self {
        Self {
            interval_us: 100_000,
            channels: [
                Some(Channel::ADV_37),
                Some(Channel::ADV_38),
                Some(Channel::ADV_39),
            ],
            connectable: true,
            scannable: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionParameters {
    pub interval_us: u32,
    pub peripheral_latency: u16,
    pub supervision_timeout_us: u32,
}

/// Explicit connection resource with generation-based stale-handle defense.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Connection {
    pub handle: u16,
    pub generation: u16,
    pub parameters: ConnectionParameters,
}

/// Position of a Link Layer procedure instant relative to `connEventCount`.
///
/// Bluetooth comparisons use modulo-65536 arithmetic. A delta in
/// `1..=32766` is in the future, zero is the current event, and a delta of
/// `32767..=65535` is considered passed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InstantRelation {
    Current,
    Future(u16),
    Passed,
}

pub const fn classify_instant(event_counter: u16, instant: u16) -> InstantRelation {
    let delta = instant.wrapping_sub(event_counter);
    match delta {
        0 => InstantRelation::Current,
        1..=32_766 => InstantRelation::Future(delta),
        _ => InstantRelation::Passed,
    }
}

/// Portable Link Layer control-PDU parsing/validation failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControlPduError {
    Malformed,
    InvalidParameters,
    InstantPassed,
}

/// Instant-bearing procedures implemented by the portable LE controller.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InstantProcedure {
    ConnectionUpdate,
    ChannelMapUpdate,
    ConnectionParametersRequest,
}

/// Core error code used when two instant-bearing procedures collide.
pub const fn procedure_collision_error(local: InstantProcedure, remote: InstantProcedure) -> u8 {
    use InstantProcedure::*;
    if matches!(
        (local, remote),
        (ConnectionUpdate, ConnectionParametersRequest)
            | (ConnectionParametersRequest, ConnectionUpdate)
    ) || local as u8 == remote as u8
    {
        0x23 // LL Procedure Collision
    } else {
        0x2a // Different Transaction Collision
    }
}

/// Parsed `LL_CONNECTION_UPDATE_IND`, including the one-byte opcode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionUpdateInd {
    pub window_size: u8,
    pub window_offset: u16,
    pub interval: u16,
    pub latency: u16,
    pub supervision_timeout: u16,
    pub instant: u16,
}

impl ConnectionUpdateInd {
    pub const OPCODE: u8 = 0x00;
    pub const PDU_LEN: usize = 12;

    pub fn parse(payload: &[u8], event_counter: u16) -> Result<Self, ControlPduError> {
        if payload.len() != Self::PDU_LEN || payload[0] != Self::OPCODE {
            return Err(ControlPduError::Malformed);
        }
        let update = Self {
            window_size: payload[1],
            window_offset: u16::from_le_bytes([payload[2], payload[3]]),
            interval: u16::from_le_bytes([payload[4], payload[5]]),
            latency: u16::from_le_bytes([payload[6], payload[7]]),
            supervision_timeout: u16::from_le_bytes([payload[8], payload[9]]),
            instant: u16::from_le_bytes([payload[10], payload[11]]),
        };
        if !update.parameters_are_valid() {
            return Err(ControlPduError::InvalidParameters);
        }
        if classify_instant(event_counter, update.instant) == InstantRelation::Passed {
            return Err(ControlPduError::InstantPassed);
        }
        Ok(update)
    }

    pub const fn parameters_are_valid(&self) -> bool {
        connection_timing_is_valid(
            self.window_size,
            self.window_offset,
            self.interval,
            self.latency,
            self.supervision_timeout,
        )
    }
}

/// Validate CONNECT_IND/LL_CONNECTION_UPDATE_IND timing fields.
///
/// Window, interval, and timeout units are respectively 1.25 ms, 1.25 ms,
/// and 10 ms. The supervision timeout must be strictly greater than twice the
/// maximum peripheral-latency event span.
pub const fn connection_timing_is_valid(
    window_size: u8,
    window_offset: u16,
    interval: u16,
    latency: u16,
    supervision_timeout: u16,
) -> bool {
    let timeout_in_interval_units = supervision_timeout as u32 * 8;
    let maximum_event_span = 2 * (latency as u32 + 1) * interval as u32;
    window_size >= 1
        && window_size <= 8
        && interval >= 6
        && interval <= 3_200
        && window_offset <= interval
        && (window_size as u16) < interval
        && latency <= 499
        && supervision_timeout >= 10
        && supervision_timeout <= 3_200
        && timeout_in_interval_units > maximum_event_span
}

/// Parsed `LL_CONNECTION_PARAM_REQ` or `LL_CONNECTION_PARAM_RSP`.
///
/// Both PDUs have the same 24-byte layout, including the one-byte opcode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConnectionParamPdu {
    pub interval_min: u16,
    pub interval_max: u16,
    pub latency: u16,
    pub supervision_timeout: u16,
    pub preferred_periodicity: u8,
    pub reference_event_count: u16,
    pub offsets: [u16; 6],
}

impl ConnectionParamPdu {
    pub const REQUEST_OPCODE: u8 = 0x0f;
    pub const RESPONSE_OPCODE: u8 = 0x10;
    pub const PDU_LEN: usize = 24;

    pub fn parse_request(payload: &[u8], event_counter: u16) -> Result<Self, ControlPduError> {
        Self::parse(payload, Self::REQUEST_OPCODE, event_counter)
    }

    pub fn parse_response(payload: &[u8], event_counter: u16) -> Result<Self, ControlPduError> {
        Self::parse(payload, Self::RESPONSE_OPCODE, event_counter)
    }

    fn parse(
        payload: &[u8],
        expected_opcode: u8,
        event_counter: u16,
    ) -> Result<Self, ControlPduError> {
        if payload.len() != Self::PDU_LEN || payload[0] != expected_opcode {
            return Err(ControlPduError::Malformed);
        }
        let mut offsets = [0u16; 6];
        for (destination, source) in offsets.iter_mut().zip(payload[12..24].chunks_exact(2)) {
            *destination = u16::from_le_bytes([source[0], source[1]]);
        }
        let parameters = Self {
            interval_min: u16::from_le_bytes([payload[1], payload[2]]),
            interval_max: u16::from_le_bytes([payload[3], payload[4]]),
            latency: u16::from_le_bytes([payload[5], payload[6]]),
            supervision_timeout: u16::from_le_bytes([payload[7], payload[8]]),
            preferred_periodicity: payload[9],
            reference_event_count: u16::from_le_bytes([payload[10], payload[11]]),
            offsets,
        };
        if parameters.parameters_are_valid(event_counter) {
            Ok(parameters)
        } else {
            Err(ControlPduError::InvalidParameters)
        }
    }

    pub const fn parameters_are_valid(&self, event_counter: u16) -> bool {
        if !connection_parameter_range_is_valid(
            self.interval_min,
            self.interval_max,
            self.latency,
            self.supervision_timeout,
        ) || self.preferred_periodicity as u16 > self.interval_max
        {
            return false;
        }

        let mut saw_invalid = false;
        let mut have_valid_offset = false;
        let mut index = 0;
        while index < self.offsets.len() {
            let offset = self.offsets[index];
            if offset == u16::MAX {
                saw_invalid = true;
            } else {
                if saw_invalid || offset >= self.interval_max {
                    return false;
                }
                let mut previous = 0;
                while previous < index {
                    if self.offsets[previous] == offset {
                        return false;
                    }
                    previous += 1;
                }
                have_valid_offset = true;
            }
            index += 1;
        }

        !have_valid_offset
            || matches!(
                classify_instant(event_counter, self.reference_event_count),
                InstantRelation::Current | InstantRelation::Future(_)
            )
    }

    /// Encode this parameter set as a request or response control PDU.
    pub fn write(&self, opcode: u8, output: &mut [u8; Self::PDU_LEN]) {
        debug_assert!(opcode == Self::REQUEST_OPCODE || opcode == Self::RESPONSE_OPCODE);
        output[0] = opcode;
        output[1..3].copy_from_slice(&self.interval_min.to_le_bytes());
        output[3..5].copy_from_slice(&self.interval_max.to_le_bytes());
        output[5..7].copy_from_slice(&self.latency.to_le_bytes());
        output[7..9].copy_from_slice(&self.supervision_timeout.to_le_bytes());
        output[9] = self.preferred_periodicity;
        output[10..12].copy_from_slice(&self.reference_event_count.to_le_bytes());
        for (destination, offset) in output[12..24].chunks_exact_mut(2).zip(self.offsets.iter()) {
            destination.copy_from_slice(&offset.to_le_bytes());
        }
    }
}

/// Validate the timing range carried by the Connection Parameters Request
/// procedure. Units are 1.25 ms for intervals and 10 ms for timeout.
pub const fn connection_parameter_range_is_valid(
    interval_min: u16,
    interval_max: u16,
    latency: u16,
    supervision_timeout: u16,
) -> bool {
    interval_min >= 6
        && interval_min <= interval_max
        && interval_max <= 3_200
        && latency <= 499
        && supervision_timeout >= 10
        && supervision_timeout <= 3_200
        && supervision_timeout as u32 * 8 > 2 * (latency as u32 + 1) * interval_max as u32
}

/// Check that a Central's update satisfies the range previously requested by
/// the Peripheral and preserves its requested latency and timeout.
pub const fn connection_update_matches_request(
    interval_min: u16,
    interval_max: u16,
    latency: u16,
    supervision_timeout: u16,
    update: ConnectionUpdateInd,
) -> bool {
    update.interval >= interval_min
        && update.interval <= interval_max
        && update.latency == latency
        && update.supervision_timeout == supervision_timeout
}

/// Parsed `LL_CHANNEL_MAP_IND`, including the one-byte opcode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ChannelMapInd {
    pub map: [u8; 5],
    pub instant: u16,
}

impl ChannelMapInd {
    pub const OPCODE: u8 = 0x01;
    pub const PDU_LEN: usize = 8;

    pub fn parse(payload: &[u8], event_counter: u16) -> Result<Self, ControlPduError> {
        if payload.len() != Self::PDU_LEN || payload[0] != Self::OPCODE {
            return Err(ControlPduError::Malformed);
        }
        let update = Self {
            map: payload[1..6].try_into().unwrap(),
            instant: u16::from_le_bytes([payload[6], payload[7]]),
        };
        if !data_channel_map_is_valid(update.map) {
            return Err(ControlPduError::InvalidParameters);
        }
        if classify_instant(event_counter, update.instant) == InstantRelation::Passed {
            return Err(ControlPduError::InstantPassed);
        }
        Ok(update)
    }
}

/// Validate the 37-bit data-channel map used by CONNECT_IND and update PDUs.
pub const fn data_channel_map_is_valid(map: [u8; 5]) -> bool {
    if map[4] & 0xe0 != 0 {
        return false;
    }
    let mut used = 0u32;
    let mut index = 0;
    while index < map.len() {
        used += map[index].count_ones();
        index += 1;
    }
    used >= 2
}

/// Computes the Data Physical Channel header for the Peripheral's
/// same-IFS encryption-start response.
///
/// The expected Central header acknowledges the outstanding cleartext
/// START_REQ and carries its current receive sequence number. Because the
/// complete expected packet is pre-authenticated before RX begins, the
/// Peripheral can also precompute the response header after committing both
/// acknowledgments and select it at the hardware RX-to-TX service point.
pub const fn encryption_start_headers(
    peripheral_transmit_sn: u8,
    peripheral_expected_sn: u8,
) -> (u8, u8) {
    let llid_control = 0x03;
    let central =
        llid_control | ((peripheral_transmit_sn ^ 1) << 2) | (peripheral_expected_sn << 3);
    let peripheral =
        llid_control | ((peripheral_expected_sn ^ 1) << 2) | ((peripheral_transmit_sn ^ 1) << 3);
    (central, peripheral)
}

/// Channel Selection Algorithm #1 state for a connection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ChannelSelection1 {
    map: [u8; 5],
    used: [u8; 37],
    used_len: u8,
    hop: u8,
    last_unmapped: u8,
}

impl ChannelSelection1 {
    pub fn new(map: [u8; 5], hop: u8) -> Result<Self, crate::Error> {
        if !(5..=16).contains(&hop) || !data_channel_map_is_valid(map) {
            return Err(crate::Error::InvalidPacket);
        }
        let mut used = [0; 37];
        let mut used_len = 0usize;
        for channel in 0..37u8 {
            if map[usize::from(channel / 8)] & (1 << (channel % 8)) != 0 {
                used[used_len] = channel;
                used_len += 1;
            }
        }
        Ok(Self {
            map,
            used,
            used_len: used_len as u8,
            hop,
            last_unmapped: 0,
        })
    }

    pub fn next_channel(&mut self) -> Channel {
        let unmapped = (self.last_unmapped + self.hop) % 37;
        self.last_unmapped = unmapped;
        let mapped = if self.map[usize::from(unmapped / 8)] & (1 << (unmapped % 8)) != 0 {
            unmapped
        } else {
            self.used[usize::from(unmapped % self.used_len)]
        };
        Channel::new(mapped).unwrap()
    }

    /// Replace the used-channel map without disturbing the CSA#1 phase.
    ///
    /// Bluetooth applies a channel-map update at an instant but explicitly
    /// keeps `lastUnmappedChannel`; rebuilding the whole selector would reset
    /// that value and desynchronize the two devices.
    pub fn update_map(&mut self, map: [u8; 5]) -> Result<(), crate::Error> {
        if !data_channel_map_is_valid(map) {
            return Err(crate::Error::InvalidPacket);
        }
        let mut used = [0; 37];
        let mut used_len = 0usize;
        for channel in 0..37u8 {
            if map[usize::from(channel / 8)] & (1 << (channel % 8)) != 0 {
                used[used_len] = channel;
                used_len += 1;
            }
        }
        self.map = map;
        self.used = used;
        self.used_len = used_len as u8;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_advertising_uses_all_primary_channels() {
        assert_eq!(
            AdvertisingConfig::default().channels,
            [
                Some(Channel::ADV_37),
                Some(Channel::ADV_38),
                Some(Channel::ADV_39)
            ]
        );
    }

    #[test]
    fn csa1_hops_and_remaps_disabled_channels() {
        let mut all = ChannelSelection1::new([0xff, 0xff, 0xff, 0xff, 0x1f], 5).unwrap();
        assert_eq!(all.next_channel().index(), 5);
        assert_eq!(all.next_channel().index(), 10);
        assert_eq!(all.next_channel().index(), 15);

        let mut sparse = ChannelSelection1::new([0b0000_0101, 0, 0, 0, 0], 5).unwrap();
        assert_eq!(sparse.next_channel().index(), 2);
        assert_eq!(sparse.next_channel().index(), 0);
    }

    #[test]
    fn map_update_preserves_csa1_unmapped_phase() {
        let mut selector = ChannelSelection1::new([0xff, 0xff, 0xff, 0xff, 0x1f], 5).unwrap();
        assert_eq!(selector.next_channel().index(), 5);
        assert_eq!(selector.next_channel().index(), 10);

        selector.update_map([0b0010_0001, 0, 0, 0, 0]).unwrap();
        // The next unmapped channel is 15. It is disabled, so 15 % 2 selects
        // the second enabled channel (5). A reset selector would select 5 only
        // by first hopping from zero and is caught by the following step.
        assert_eq!(selector.next_channel().index(), 5);
        assert_eq!(selector.next_channel().index(), 0);
        assert!(selector.update_map([1, 0, 0, 0, 0]).is_err());
    }

    #[test]
    fn encryption_start_headers_commit_both_acknowledgments() {
        // Peripheral START_REQ used SN=0 and had already accepted Central
        // SN=0, so it expects SN=1. Central's 0x0f acknowledges both. The
        // anticipated response therefore commits NESN=0 and new SN=1.
        assert_eq!(encryption_start_headers(0, 1), (0x0f, 0x0b));
    }

    #[test]
    fn data_sequence_accepts_new_pdus_and_rejects_retransmissions() {
        let mut sequence = DataSequenceState::new(0, 0);

        // Central SN=0 is new, but NESN=0 cannot acknowledge a Peripheral
        // packet which still uses SN=0.
        assert_eq!(
            sequence.observe(0x01),
            DataSequenceObservation {
                acknowledged_transmit: false,
                accepted_receive: true,
                peer_more_data: false,
            }
        );
        assert_eq!(sequence, DataSequenceState::new(0, 1));

        // Repeating SN=0 is a duplicate. Advancing NESN acknowledges the
        // outstanding Peripheral packet and prepares SN=1 for its next PDU.
        assert_eq!(
            sequence.observe(0x01 | DATA_HEADER_NESN),
            DataSequenceObservation {
                acknowledged_transmit: true,
                accepted_receive: false,
                peer_more_data: false,
            }
        );
        assert_eq!(sequence, DataSequenceState::new(1, 1));

        // Both sequence numbers wrap independently after the next new PDU.
        let observation = sequence.observe(0x01 | DATA_HEADER_NESN | DATA_HEADER_SN);
        assert!(!observation.acknowledged_transmit);
        assert!(observation.accepted_receive);
        assert_eq!(sequence, DataSequenceState::new(1, 0));
    }

    #[test]
    fn peripheral_entry_packet_cannot_ack_a_fresh_response() {
        let peer_advanced_nesn = 0x01 | DATA_HEADER_NESN;
        assert!(!acknowledges_outstanding_transmit(
            peer_advanced_nesn,
            0,
            false
        ));
        assert!(acknowledges_outstanding_transmit(
            peer_advanced_nesn,
            0,
            true
        ));
        assert!(!acknowledges_transmit_owner(
            peer_advanced_nesn,
            0,
            false,
            false,
        ));
        assert!(acknowledges_transmit_owner(
            peer_advanced_nesn,
            0,
            false,
            true,
        ));
    }

    #[test]
    fn response_header_preserves_llid_and_reserved_fields() {
        let sequence = DataSequenceState::new(1, 0);
        let staged = 0xe3 | DATA_HEADER_NESN | DATA_HEADER_MD;
        assert_eq!(sequence.response_header(staged, false), 0xeb);
        assert_eq!(sequence.response_header(staged, true), 0xfb);
    }

    #[test]
    fn gate_b_four_leg_empty_closure_preserves_next_event_ownership() {
        // Peripheral starts with SN=0 and expects the Central's SN=1.
        let mut sequence = DataSequenceState::new(0, 1);

        // Central Write Command: NESN=1 acknowledges the previous Peripheral
        // Empty and SN=1 supplies the new request.
        let request = sequence.observe(0x0e);
        assert!(request.acknowledged_transmit);
        assert!(request.accepted_receive);
        assert_eq!(sequence, DataSequenceState::new(1, 0));

        // The same-event ATT Notification advertises MD and owns SN=1/NESN=0.
        assert_eq!(sequence.response_header(0x02, true), 0x1a);

        // Central's new Empty PDU acknowledges that response and must itself
        // advance the Peripheral receive expectation.
        let central_empty = sequence.observe(0x01);
        assert!(central_empty.acknowledged_transmit);
        assert!(central_empty.accepted_receive);
        assert_eq!(sequence, DataSequenceState::new(0, 1));

        // The closing Peripheral Empty is now the sole outstanding TX owner.
        assert_eq!(sequence.response_header(0x01, false), 0x05);

        // Next event's Write uses SN=1 and NESN=1, simultaneously accepting
        // that closing Empty and supplying the next request.
        let next_request = sequence.observe(0x0e);
        assert!(next_request.acknowledged_transmit);
        assert!(next_request.accepted_receive);
        assert_eq!(sequence, DataSequenceState::new(1, 0));
    }

    #[test]
    fn md_from_either_participant_allows_event_continuation() {
        assert!(!connection_event_may_continue(false, false));
        assert!(connection_event_may_continue(true, false));
        assert!(connection_event_may_continue(false, true));
        assert!(connection_event_may_continue(true, true));

        let mut sequence = DataSequenceState::new(0, 0);
        let observation = sequence.observe(0x01 | DATA_HEADER_NESN | DATA_HEADER_MD);
        assert!(observation.peer_more_data);
    }

    #[test]
    fn final_fresh_owner_requests_exactly_one_ack_leg() {
        assert!(response_advertises_more_data(
            false, true, true, false, true
        ));
        assert!(!response_advertises_more_data(
            false, true, true, true, true
        ));
        assert!(!response_advertises_more_data(
            false, true, false, false, true
        ));
        assert!(!response_advertises_more_data(
            false, true, true, false, false
        ));
        assert!(response_advertises_more_data(
            true, false, true, false, true
        ));
    }

    #[test]
    fn event_decision_prioritizes_integrity_deadline_and_resources() {
        let mut input = ConnectionEventDecisionInput {
            crc_ok: true,
            mic_ok: true,
            local_more_data: true,
            peer_more_data: true,
            before_deadline: true,
            resources_available: true,
        };
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::Continue
        );

        input.crc_ok = false;
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::InvalidCrc
        );
        input.crc_ok = true;
        input.mic_ok = false;
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::InvalidMic
        );
        input.mic_ok = true;
        input.before_deadline = false;
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::Deadline
        );
        input.before_deadline = true;
        input.local_more_data = false;
        input.peer_more_data = false;
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::BothMdClear
        );
        input.peer_more_data = true;
        input.resources_available = false;
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::ResourceLimit
        );
    }

    #[test]
    fn event_decision_keeps_two_four_and_eight_pdu_sequences_open() {
        for exchanges in [2usize, 4, 8] {
            let mut sequence = DataSequenceState::new(0, 0);
            for index in 0..exchanges {
                let final_exchange = index + 1 == exchanges;
                let header = 0x01
                    | (sequence.transmit_sn << 2)
                    | (sequence.expected_sn << 3)
                    | if final_exchange { 0 } else { DATA_HEADER_MD };
                let observation = sequence.observe(header);
                let decision = connection_event_decision(ConnectionEventDecisionInput {
                    crc_ok: true,
                    mic_ok: true,
                    local_more_data: false,
                    peer_more_data: observation.peer_more_data,
                    before_deadline: true,
                    resources_available: true,
                });
                assert_eq!(
                    decision,
                    if final_exchange {
                        ConnectionEventCloseReason::BothMdClear
                    } else {
                        ConnectionEventCloseReason::Continue
                    }
                );
            }
        }
    }

    #[test]
    fn rolling_tx_slots_retry_until_nesn_acknowledges_each_payload() {
        let mut transmit = ConnectionEventTxState::new(2);
        let first = transmit.select(false).unwrap();
        assert_eq!(first.response_slot, ConnectionEventTxSlot::Slot0);
        assert_eq!(first.response_token.unwrap().payload_id, 0);
        assert!(!first.acknowledged_response);
        transmit.complete_transmit(first).unwrap();

        // Lost ACK: payload zero and its SN remain owned by physical slot 0.
        let retry = transmit.select(false).unwrap();
        assert_eq!(retry.response_slot, ConnectionEventTxSlot::Slot0);
        assert_eq!(retry.response_token.unwrap().payload_id, 0);
        assert!(!retry.acknowledged_response);
        transmit.complete_transmit(retry).unwrap();

        let second = transmit.select(true).unwrap();
        assert_eq!(second.response_slot, ConnectionEventTxSlot::Slot1);
        assert_eq!(second.response_token.unwrap().payload_id, 1);
        assert_eq!(second.acknowledged_token.unwrap().payload_id, 0);
        transmit.complete_transmit(second).unwrap();

        // Lost ACK for payload one cannot free or replace it.
        let second_retry = transmit.select(false).unwrap();
        assert_eq!(second_retry.response_slot, ConnectionEventTxSlot::Slot1);
        assert_eq!(second_retry.response_token.unwrap().payload_id, 1);
        assert!(!second_retry.acknowledged_response);
        transmit.complete_transmit(second_retry).unwrap();

        let empty = transmit.select(true).unwrap();
        assert_eq!(empty.response_slot, ConnectionEventTxSlot::Slot1);
        assert_eq!(empty.response_token, None);
        assert_eq!(empty.acknowledged_token.unwrap().payload_id, 1);
        transmit.complete_transmit(empty).unwrap();
        let empty_retry = transmit.select(false).unwrap();
        assert_eq!(empty_retry.response_slot, ConnectionEventTxSlot::Slot1);
        assert_eq!(empty_retry.response_token, None);
    }

    #[test]
    fn first_incoming_nesn_cannot_ack_a_not_yet_transmitted_slot() {
        let mut transmit = ConnectionEventTxState::new(2);
        assert_eq!(
            transmit.select(true),
            Err(ConnectionEventTxError::AckWithoutOutstandingResponse)
        );
    }

    #[test]
    fn empty_retry_does_not_consume_or_ack_a_new_host_payload() {
        for owners in [&[][..], &[10][..], &[10, 20][..]] {
            let mut tx = ConnectionEventTxState::with_empty_retry(owners).unwrap();
            let retry = tx.select(false).unwrap();
            assert_eq!(retry.response_token, None);
            assert!(retry.response_is_retry);
            assert!(retry.refill_after_transmit.is_none());
            tx.complete_transmit(retry).unwrap();
            let first = tx.select(true).unwrap();
            assert!(first.acknowledged_response);
            assert_eq!(first.acknowledged_token, None);
            assert_eq!(
                first.response_token.map(|t| t.owner_token),
                owners.first().copied()
            );
            assert_eq!(
                first.refill_after_transmit.map(|r| r.token.owner_token),
                owners.get(1).copied()
            );
            tx.complete_transmit(first).unwrap();
            let retry = tx.select(false).unwrap();
            assert_eq!(retry.response_token, first.response_token);
            tx.complete_transmit(retry).unwrap();
            if owners.len() == 2 {
                let second = tx.select(true).unwrap();
                assert_eq!(second.acknowledged_token.unwrap().owner_token, 10);
                assert_eq!(second.response_token.unwrap().owner_token, 20);
                tx.complete_transmit(second).unwrap();
                let final_ack = tx.select(true).unwrap();
                assert_eq!(final_ack.acknowledged_token.unwrap().owner_token, 20);
                assert_eq!(final_ack.response_token, None);
            }
        }
    }

    #[test]
    fn two_host_payloads_follow_a_closing_empty_without_a_third_event() {
        let mut sequence = DataSequenceState::new(1, 0);
        let mut tx = ConnectionEventTxState::with_empty_retry(&[10, 20]).unwrap();
        // Previous CE received both requests and closed with Empty SN=1.
        // This CE starts with Central ACK of that Empty, not payload 10.
        let first = sequence.observe(0x01);
        let response = tx.select(first.acknowledged_transmit).unwrap();
        assert_eq!(response.acknowledged_token, None);
        assert_eq!(response.response_token.unwrap().owner_token, 10);
        assert_eq!(sequence.response_header(0x02, true), 0x16);
        tx.complete_transmit(response).unwrap();
        // Central ACKs payload 10 in the same event. Consume queued payload
        // 20, advance each sequence exactly once, and clear MD on the last.
        let second = sequence.observe(0x0d);
        let response = tx.select(second.acknowledged_transmit).unwrap();
        assert_eq!(response.acknowledged_token.unwrap().owner_token, 10);
        assert_eq!(response.response_token.unwrap().owner_token, 20);
        assert_eq!(sequence.response_header(0x02, false), 0x0a);
        tx.complete_transmit(response).unwrap();
        let retry = tx.select(false).unwrap();
        assert_eq!(retry.response_token.unwrap().owner_token, 20);
        assert!(retry.response_is_retry);
    }

    #[test]
    fn aborted_first_payload_after_empty_ack_keeps_payload_zero_staged() {
        let mut tx = ConnectionEventTxState::with_empty_retry(&[10, 20]).unwrap();
        let first = tx.select(true).unwrap();
        assert_eq!(first.acknowledged_token, None);
        tx.abort_transmit(first).unwrap();
        // Empty's ACK was real; no Host TX has completed. The next unchanged
        // NESN must retry the intent without acknowledging payload zero.
        let retry = tx.select(false).unwrap();
        assert!(!retry.acknowledged_response);
        assert!(!retry.response_is_retry);
        assert_eq!(retry.response_token, first.response_token);
        tx.complete_transmit(retry).unwrap();
        let second = tx.select(true).unwrap();
        assert_eq!(second.acknowledged_token.unwrap().owner_token, 10);
        assert_eq!(second.response_token.unwrap().owner_token, 20);
    }

    #[test]
    fn response_plan_can_commit_or_abort_exactly_once() {
        let mut committed = ConnectionEventTxState::new(2);
        let decision = committed.select(false).unwrap();
        assert_eq!(
            committed.select(false),
            Err(ConnectionEventTxError::ResponsePlanAlreadyPending)
        );
        committed.complete_transmit(decision).unwrap();
        assert_eq!(
            committed.complete_transmit(decision),
            Err(ConnectionEventTxError::CompletionDoesNotMatchDecision)
        );

        let mut aborted = ConnectionEventTxState::new(2);
        let decision = aborted.select(false).unwrap();
        aborted.abort_transmit(decision).unwrap();
        assert_eq!(
            aborted.complete_transmit(decision),
            Err(ConnectionEventTxError::CompletionDoesNotMatchDecision)
        );
    }

    #[test]
    fn preview_is_not_a_transmit_grant() {
        let mut state = ConnectionEventTxState::with_empty_retry(&[10, 20]).unwrap();
        let before = state;
        let next = state.preview_select(true).unwrap();
        assert_eq!(state, before);
        assert_eq!(next.response_token.unwrap().owner_token, 10);
        assert_eq!(
            state.complete_transmit(next),
            Err(ConnectionEventTxError::CompletionDoesNotMatchDecision)
        );
        let retry = state.select(false).unwrap();
        assert!(retry.response_is_retry);
        assert_eq!(retry.response_token, None);
        assert_eq!(
            state.preview_select(true),
            Err(ConnectionEventTxError::ResponsePlanAlreadyPending)
        );
        state.complete_transmit(retry).unwrap();
        let selected = state.select(true).unwrap();
        assert_eq!(selected, next);
        state.abort_transmit(selected).unwrap();
        let fresh = state.preview_select(false).unwrap();
        assert!(!fresh.response_is_retry);
        assert_eq!(fresh.response_token, next.response_token);
    }

    #[test]
    fn preview_matches_selection_across_retry_abort_and_refill() {
        for count in 0..=MAX_CONNECTION_EVENT_TX_PAYLOADS {
            for empty in [false, true] {
                let owners = [101, 202, 303, 404, 505, 606, 707, 808];
                let mut state = if empty {
                    ConnectionEventTxState::with_empty_retry(&owners[..count]).unwrap()
                } else {
                    ConnectionEventTxState::new_with_owner_tokens(&owners[..count]).unwrap()
                };
                let mut random = 0x585582_u32;
                for _ in 0..512 {
                    for ack in [false, true] {
                        let before = state;
                        let preview = state.preview_select(ack);
                        let mut selected = state;
                        assert_eq!(preview, selected.select(ack));
                        assert_eq!(state, before);
                    }
                    random ^= random << 13;
                    random ^= random >> 17;
                    random ^= random << 5;
                    if let Ok(decision) = state.select(random & 1 != 0) {
                        if random & 6 == 0 {
                            state.abort_transmit(decision).unwrap();
                        } else {
                            state.complete_transmit(decision).unwrap();
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn aborted_refill_does_not_skip_payload_or_generation() {
        let mut transmit = ConnectionEventTxState::new(4);
        let first = transmit.select(false).unwrap();
        transmit.complete_transmit(first).unwrap();

        let aborted = transmit.select(true).unwrap();
        let refill = aborted.refill_after_transmit.unwrap();
        assert_eq!(refill.token.payload_id, 2);
        assert_eq!(refill.token.generation, 1);
        transmit.abort_transmit(aborted).unwrap();

        let retried = transmit.select(false).unwrap();
        let retried_refill = retried.refill_after_transmit.unwrap();
        assert_eq!(retried.response_token.unwrap().payload_id, 1);
        assert_eq!(retried_refill.token.payload_id, 2);
        assert_eq!(retried_refill.token.generation, 1);
        transmit.complete_transmit(retried).unwrap();

        let next = transmit.select(true).unwrap();
        assert_eq!(next.acknowledged_token.unwrap().payload_id, 1);
        assert_eq!(next.response_token.unwrap().payload_id, 2);
    }

    #[test]
    fn controller_owner_token_survives_slot_roll_and_retry() {
        let owners = [0xA001u32, 0xB002, 0xC003, 0xD004];
        let mut transmit = ConnectionEventTxState::new_with_owner_tokens(&owners).unwrap();
        let first = transmit.select(false).unwrap();
        assert_eq!(first.response_token.unwrap().owner_token, owners[0]);
        transmit.complete_transmit(first).unwrap();

        let retry = transmit.select(false).unwrap();
        assert_eq!(retry.response_token.unwrap().owner_token, owners[0]);
        assert!(retry.response_is_retry);
        transmit.complete_transmit(retry).unwrap();

        for expected in 0..owners.len() {
            let decision = transmit.select(true).unwrap();
            assert_eq!(
                decision.acknowledged_token.unwrap().owner_token,
                owners[expected]
            );
            assert_eq!(
                decision.response_token.map(|token| token.owner_token),
                owners.get(expected + 1).copied()
            );
            transmit.complete_transmit(decision).unwrap();
        }
    }

    #[test]
    fn rolling_slots_preserve_payload_identity_for_two_four_and_eight_pdus() {
        for payload_count in [2u8, 4, 8] {
            let mut transmit = ConnectionEventTxState::new(payload_count);
            let first = transmit.select(false).unwrap();
            assert_eq!(first.response_token.unwrap().payload_id, 0);
            assert!(!first.response_is_retry);
            transmit.complete_transmit(first).unwrap();

            for expected_ack in 0..payload_count {
                let decision = transmit.select(true).unwrap();
                assert_eq!(
                    decision.acknowledged_token.map(|token| token.payload_id),
                    Some(expected_ack)
                );
                assert_eq!(
                    decision.response_token.map(|token| token.payload_id),
                    (expected_ack + 1 < payload_count).then_some(expected_ack + 1)
                );
                assert!(!decision.response_is_retry);
                if let Some(refill) = decision.refill_after_transmit {
                    assert_eq!(refill.token.payload_id, expected_ack + 2);
                    assert_ne!(refill.slot, decision.response_slot);
                }
                transmit.complete_transmit(decision).unwrap();
            }
        }
    }

    #[test]
    fn ack_loss_never_releases_or_refills_the_outstanding_payload() {
        let mut transmit = ConnectionEventTxState::new(4);
        let first = transmit.select(false).unwrap();
        assert_eq!(first.response_token.unwrap().payload_id, 0);
        transmit.complete_transmit(first).unwrap();
        for _ in 0..3 {
            let retry = transmit.select(false).unwrap();
            assert_eq!(retry.response_token.unwrap().payload_id, 0);
            assert!(retry.response_is_retry);
            assert!(!retry.acknowledged_response);
            assert_eq!(retry.refill_after_transmit, None);
            transmit.complete_transmit(retry).unwrap();
        }
        let advance = transmit.select(true).unwrap();
        assert_eq!(advance.acknowledged_token.unwrap().payload_id, 0);
        assert_eq!(advance.response_token.unwrap().payload_id, 1);
        assert_eq!(
            advance.refill_after_transmit,
            Some(ConnectionEventTxRefill {
                slot: ConnectionEventTxSlot::Slot0,
                token: ConnectionEventTxToken {
                    payload_id: 2,
                    owner_token: 2,
                    generation: 1,
                },
            })
        );
    }

    #[test]
    fn ack_loss_at_every_position_retries_the_exact_token_for_two_four_and_eight_pdus() {
        for payload_count in [2u8, 4, 8] {
            for lost_payload_id in 0..payload_count {
                let mut transmit = ConnectionEventTxState::new(payload_count);
                let first = transmit.select(false).unwrap();
                transmit.complete_transmit(first).unwrap();

                for payload_id in 0..payload_count {
                    if payload_id == lost_payload_id {
                        let retry = transmit.select(false).unwrap();
                        assert_eq!(retry.response_token.unwrap().payload_id, payload_id);
                        assert!(retry.response_is_retry);
                        assert!(!retry.acknowledged_response);
                        assert_eq!(retry.acknowledged_token, None);
                        assert_eq!(retry.refill_after_transmit, None);
                        transmit.complete_transmit(retry).unwrap();
                    }

                    let advance = transmit.select(true).unwrap();
                    assert_eq!(
                        advance.acknowledged_token.map(|token| token.payload_id),
                        Some(payload_id)
                    );
                    transmit.complete_transmit(advance).unwrap();
                }
            }
        }
    }

    #[test]
    fn failed_response_keeps_the_outstanding_llid_for_retry() {
        let outstanding = 0x13;
        assert_eq!(
            committed_response_header(outstanding, 0, false),
            outstanding
        );
        assert_eq!(committed_response_header(outstanding, 0x1b, true), 0x1b);
    }

    #[test]
    fn duplicate_central_pdus_are_acked_but_delivered_once() {
        let mut sequence = DataSequenceState::new(0, 0);
        let mut deliveries = 0;
        for header in [0x01 | DATA_HEADER_MD, 0x01 | DATA_HEADER_MD, 0x09] {
            let observation = sequence.observe(header);
            deliveries += usize::from(observation.accepted_receive);
        }
        assert_eq!(deliveries, 2);
        assert_eq!(sequence.expected_sn, 0);
    }

    #[test]
    fn lost_ack_keeps_transmit_sn_for_the_following_event() {
        let mut sequence = DataSequenceState::new(1, 0);
        let before = sequence.transmit_sn;
        let observation = sequence.observe(0x01 | DATA_HEADER_NESN | DATA_HEADER_MD);
        assert!(!observation.acknowledged_transmit);
        assert_eq!(sequence.transmit_sn, before);

        // A new event starts with the same protocol state. Only a peer NESN
        // transition can release the retry and advance the local SN.
        let mut following_event = sequence;
        let observation = following_event.observe(0x01);
        assert!(observation.acknowledged_transmit);
        assert_eq!(following_event.transmit_sn, before ^ 1);
    }

    #[test]
    fn eighth_receive_slot_closes_with_resource_limit_and_no_md_promise() {
        let input = ConnectionEventDecisionInput {
            crc_ok: true,
            mic_ok: true,
            local_more_data: true,
            peer_more_data: true,
            before_deadline: true,
            resources_available: false,
        };
        assert_eq!(
            connection_event_decision(input),
            ConnectionEventCloseReason::ResourceLimit
        );
        let state = DataSequenceState::new(1, 1);
        assert_eq!(
            state.response_header(0x03 | DATA_HEADER_MD, false) & DATA_HEADER_MD,
            0
        );
    }

    #[test]
    fn connection_event_trace_encoding_preserves_status_sequence_and_close_reason() {
        let record = ConnectionEventTraceRecord {
            event_counter: 0x1234,
            channel: 37,
            leg_index: 7,
            timestamp_us: 0x0123_4567_89ab_cdef,
            lle_status: 0x2005,
            bb_status: 0x0060,
            peer_header: 0x1d,
            response_header: 0x15,
            received_len: 31,
            response_slot: 2,
            crc_ok: true,
            acknowledged_transmit: true,
            accepted_receive: false,
            peer_more_data: true,
            response_transmitted: true,
            close_reason: ConnectionEventCloseReason::ResourceLimit,
        };
        assert_eq!(
            record.encode(),
            [
                0x0725_1234,
                0x89ab_cdef,
                0x0123_4567,
                0x2005,
                0x0060,
                0x021f_151d,
                0x061b,
            ]
        );
        assert_eq!(connection_event_trace_slot(0, 16), 0);
        assert_eq!(connection_event_trace_slot(15, 16), 15);
        assert_eq!(connection_event_trace_slot(16, 16), 0);
        assert_eq!(connection_event_trace_slot(u32::MAX, 16), 15);
    }

    #[test]
    fn crc_failure_does_not_advance_sequence_state() {
        let sequence = DataSequenceState::new(1, 0);
        let before = sequence;
        let decision = connection_event_decision(ConnectionEventDecisionInput {
            crc_ok: false,
            mic_ok: true,
            local_more_data: true,
            peer_more_data: true,
            before_deadline: true,
            resources_available: true,
        });
        assert_eq!(decision, ConnectionEventCloseReason::InvalidCrc);
        assert_eq!(sequence, before);
    }

    #[test]
    fn window_widening_uses_both_sca_values_and_rounds_up() {
        assert_eq!(sleep_clock_accuracy_ppm(0), 500);
        assert_eq!(sleep_clock_accuracy_ppm(7), 20);
        assert_eq!(window_widening_us(7_500, 0, 50), 5);
        assert_eq!(window_widening_us(1_000_000, 5, 50), 100);
        assert_eq!(window_widening_us(u64::MAX, 0, 500), 18_446_744_073_709_552);
    }

    #[test]
    fn instant_classification_covers_current_boundary_and_wraparound() {
        assert_eq!(classify_instant(100, 100), InstantRelation::Current);
        assert_eq!(classify_instant(100, 101), InstantRelation::Future(1));
        assert_eq!(
            classify_instant(100, 100u16.wrapping_add(32_766)),
            InstantRelation::Future(32_766)
        );
        assert_eq!(
            classify_instant(100, 100u16.wrapping_add(32_767)),
            InstantRelation::Passed
        );
        assert_eq!(classify_instant(0xfffe, 1), InstantRelation::Future(3));
        assert_eq!(classify_instant(1, 0xfffe), InstantRelation::Passed);
    }

    #[test]
    fn collision_codes_distinguish_same_and_different_instant_procedures() {
        assert_eq!(
            procedure_collision_error(
                InstantProcedure::ConnectionParametersRequest,
                InstantProcedure::ConnectionParametersRequest,
            ),
            0x23
        );
        assert_eq!(
            procedure_collision_error(
                InstantProcedure::ConnectionParametersRequest,
                InstantProcedure::ConnectionUpdate,
            ),
            0x23
        );
        assert_eq!(
            procedure_collision_error(
                InstantProcedure::ConnectionParametersRequest,
                InstantProcedure::ChannelMapUpdate,
            ),
            0x2a
        );
    }

    #[test]
    fn data_access_address_enforces_core_bit_patterns() {
        assert!(data_access_address_is_valid(0x1234_5678, false));
        assert!(!data_access_address_is_valid(ADV_ACCESS_ADDRESS, false));
        assert!(!data_access_address_is_valid(ADV_ACCESS_ADDRESS ^ 1, false));
        assert!(!data_access_address_is_valid(0x1111_1111, false));
        assert!(!data_access_address_is_valid(0x0000_007f, false));
        assert!(!data_access_address_is_valid(0xaaaa_aaab, false));

        // Valid on LE 1M but not on LE Coded: the least-significant octet has
        // fewer than three ones.
        assert!(data_access_address_is_valid(0x1234_5640, false));
        assert!(!data_access_address_is_valid(0x1234_5640, true));
    }

    fn connection_update_payload(
        window_size: u8,
        window_offset: u16,
        interval: u16,
        latency: u16,
        timeout: u16,
        instant: u16,
    ) -> [u8; ConnectionUpdateInd::PDU_LEN] {
        let mut payload = [0; ConnectionUpdateInd::PDU_LEN];
        payload[0] = ConnectionUpdateInd::OPCODE;
        payload[1] = window_size;
        payload[2..4].copy_from_slice(&window_offset.to_le_bytes());
        payload[4..6].copy_from_slice(&interval.to_le_bytes());
        payload[6..8].copy_from_slice(&latency.to_le_bytes());
        payload[8..10].copy_from_slice(&timeout.to_le_bytes());
        payload[10..12].copy_from_slice(&instant.to_le_bytes());
        payload
    }

    #[test]
    fn connection_update_accepts_current_and_wrapped_future_instants() {
        let current = connection_update_payload(1, 0, 6, 0, 10, 42);
        assert_eq!(
            ConnectionUpdateInd::parse(&current, 42).unwrap().instant,
            42
        );

        let wrapped = connection_update_payload(1, 0, 12, 0, 20, 2);
        assert_eq!(
            ConnectionUpdateInd::parse(&wrapped, 0xfffe)
                .unwrap()
                .instant,
            2
        );
    }

    #[test]
    fn connection_update_rejects_malformed_invalid_and_passed_values() {
        let valid = connection_update_payload(1, 0, 6, 0, 10, 12);
        assert_eq!(
            ConnectionUpdateInd::parse(&valid[..11], 6),
            Err(ControlPduError::Malformed)
        );

        for invalid in [
            connection_update_payload(0, 0, 6, 0, 10, 12),
            connection_update_payload(6, 0, 6, 0, 10, 12),
            connection_update_payload(1, 7, 6, 0, 10, 12),
            connection_update_payload(1, 0, 5, 0, 10, 12),
            connection_update_payload(1, 0, 6, 500, 1_000, 12),
            // Equality is invalid: 100 ms must be strictly greater than
            // twice a 50-ms interval.
            connection_update_payload(1, 0, 40, 0, 10, 12),
        ] {
            assert_eq!(
                ConnectionUpdateInd::parse(&invalid, 6),
                Err(ControlPduError::InvalidParameters)
            );
        }

        let passed = connection_update_payload(1, 0, 6, 0, 10, 5);
        assert_eq!(
            ConnectionUpdateInd::parse(&passed, 6),
            Err(ControlPduError::InstantPassed)
        );
    }

    fn connection_param_payload(parameters: ConnectionParamPdu, opcode: u8) -> [u8; 24] {
        let mut payload = [0; ConnectionParamPdu::PDU_LEN];
        parameters.write(opcode, &mut payload);
        payload
    }

    fn valid_connection_params() -> ConnectionParamPdu {
        ConnectionParamPdu {
            interval_min: 12,
            interval_max: 24,
            latency: 0,
            supervision_timeout: 200,
            preferred_periodicity: 6,
            reference_event_count: 102,
            offsets: [3, 7, u16::MAX, u16::MAX, u16::MAX, u16::MAX],
        }
    }

    #[test]
    fn connection_parameter_request_round_trips_and_accepts_wraparound_reference() {
        let mut parameters = valid_connection_params();
        parameters.reference_event_count = 1;
        let request = connection_param_payload(parameters, ConnectionParamPdu::REQUEST_OPCODE);
        assert_eq!(
            ConnectionParamPdu::parse_request(&request, 0xfffe),
            Ok(parameters)
        );

        let response = connection_param_payload(parameters, ConnectionParamPdu::RESPONSE_OPCODE);
        assert_eq!(
            ConnectionParamPdu::parse_response(&response, 0xfffe),
            Ok(parameters)
        );
        assert_eq!(
            ConnectionParamPdu::parse_request(&response, 0xfffe),
            Err(ControlPduError::Malformed)
        );
    }

    #[test]
    fn connection_parameter_request_validates_timing_and_anchor_preferences() {
        let valid = valid_connection_params();
        for invalid in [
            ConnectionParamPdu {
                interval_min: 5,
                ..valid
            },
            ConnectionParamPdu {
                interval_min: 25,
                ..valid
            },
            ConnectionParamPdu {
                latency: 500,
                ..valid
            },
            ConnectionParamPdu {
                // 100 ms is not greater than twice the 50-ms maximum interval.
                interval_max: 40,
                supervision_timeout: 10,
                ..valid
            },
            ConnectionParamPdu {
                preferred_periodicity: 25,
                ..valid
            },
            ConnectionParamPdu {
                offsets: [24, u16::MAX, u16::MAX, u16::MAX, u16::MAX, u16::MAX],
                ..valid
            },
            ConnectionParamPdu {
                offsets: [3, u16::MAX, 7, u16::MAX, u16::MAX, u16::MAX],
                ..valid
            },
            ConnectionParamPdu {
                offsets: [3, 3, u16::MAX, u16::MAX, u16::MAX, u16::MAX],
                ..valid
            },
            ConnectionParamPdu {
                reference_event_count: 99,
                ..valid
            },
        ] {
            let payload = connection_param_payload(invalid, ConnectionParamPdu::REQUEST_OPCODE);
            assert_eq!(
                ConnectionParamPdu::parse_request(&payload, 100),
                Err(ControlPduError::InvalidParameters)
            );
        }

        // With no valid offset, ReferenceConnEventCount is ignored.
        let no_offsets = ConnectionParamPdu {
            reference_event_count: 99,
            offsets: [u16::MAX; 6],
            ..valid
        };
        let payload = connection_param_payload(no_offsets, ConnectionParamPdu::REQUEST_OPCODE);
        assert_eq!(
            ConnectionParamPdu::parse_request(&payload, 100),
            Ok(no_offsets)
        );
    }

    #[test]
    fn connection_update_must_satisfy_the_peripheral_request() {
        let matching = ConnectionUpdateInd {
            window_size: 1,
            window_offset: 0,
            interval: 18,
            latency: 4,
            supervision_timeout: 200,
            instant: 10,
        };
        assert!(connection_update_matches_request(12, 24, 4, 200, matching));
        for update in [
            ConnectionUpdateInd {
                interval: 11,
                ..matching
            },
            ConnectionUpdateInd {
                interval: 25,
                ..matching
            },
            ConnectionUpdateInd {
                latency: 3,
                ..matching
            },
            ConnectionUpdateInd {
                supervision_timeout: 201,
                ..matching
            },
        ] {
            assert!(!connection_update_matches_request(12, 24, 4, 200, update));
        }
    }

    #[test]
    fn channel_map_update_validates_map_and_instant() {
        let mut payload = [0u8; ChannelMapInd::PDU_LEN];
        payload[0] = ChannelMapInd::OPCODE;
        payload[1..6].copy_from_slice(&[0x05, 0, 0, 0, 0]);
        payload[6..8].copy_from_slice(&2u16.to_le_bytes());
        assert_eq!(
            ChannelMapInd::parse(&payload, 0xfffe).unwrap().map,
            [0x05, 0, 0, 0, 0]
        );

        payload[1..6].copy_from_slice(&[0x01, 0, 0, 0, 0]);
        assert_eq!(
            ChannelMapInd::parse(&payload, 0xfffe),
            Err(ControlPduError::InvalidParameters)
        );
        payload[1..6].copy_from_slice(&[0x05, 0, 0, 0, 0xe0]);
        assert_eq!(
            ChannelMapInd::parse(&payload, 0xfffe),
            Err(ControlPduError::InvalidParameters)
        );
        payload[1..6].copy_from_slice(&[0x05, 0, 0, 0, 0]);
        payload[6..8].copy_from_slice(&0xfffdu16.to_le_bytes());
        assert_eq!(
            ChannelMapInd::parse(&payload, 0xfffe),
            Err(ControlPduError::InstantPassed)
        );
    }
}

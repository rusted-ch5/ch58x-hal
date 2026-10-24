//! Portable ownership state for Link Layer control procedures.
//!
//! A hardware backend may queue and transmit Control PDUs, but it must not
//! infer procedure completion from queue occupancy. This module keeps local
//! and remote procedure ownership separate and advances them only from exact
//! TX, NESN, response, timeout and instant events.

use crate::link_layer::{
    InstantProcedure, InstantRelation, classify_instant, procedure_collision_error,
};

/// Control procedures implemented by the compact keyboard controller.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcedureKind {
    Termination,
    FeatureExchange,
    VersionExchange,
    ConnectionParameters,
    ConnectionUpdate,
    ChannelMapUpdate,
    Ping,
}

impl ProcedureKind {
    const fn instant_procedure(self) -> Option<InstantProcedure> {
        match self {
            Self::ConnectionParameters => Some(InstantProcedure::ConnectionParametersRequest),
            Self::ConnectionUpdate => Some(InstantProcedure::ConnectionUpdate),
            Self::ChannelMapUpdate => Some(InstantProcedure::ChannelMapUpdate),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcedureInitiator {
    Local,
    Remote,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcedurePhase {
    Queued,
    AwaitingTxAck,
    AwaitingResponse,
    ResponseQueued,
    AwaitingInstant,
    Complete,
    Failed,
}

/// Exact logical Control PDU identity. Generation rejects a late completion
/// after a physical slot has been recycled.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProcedureToken {
    pub owner_token: u32,
    pub generation: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProcedureContext {
    pub kind: ProcedureKind,
    pub initiator: ProcedureInitiator,
    pub phase: ProcedurePhase,
    pub token: ProcedureToken,
    pub tx_opcode: u8,
    pub expected_response_opcode: Option<u8>,
    pub instant: Option<u16>,
    response_timeout_us: u32,
    response_deadline_us: u64,
    pub pending_response: bool,
    pub error: Option<u8>,
}

impl ProcedureContext {
    pub const fn new(
        kind: ProcedureKind,
        initiator: ProcedureInitiator,
        token: ProcedureToken,
        tx_opcode: u8,
        expected_response_opcode: Option<u8>,
        instant: Option<u16>,
        response_timeout_us: Option<u32>,
    ) -> Self {
        Self {
            kind,
            initiator,
            phase: ProcedurePhase::Queued,
            token,
            tx_opcode,
            expected_response_opcode,
            instant,
            response_timeout_us: match response_timeout_us {
                Some(timeout) => timeout,
                None => 0,
            },
            response_deadline_us: 0,
            pending_response: false,
            error: None,
        }
    }

    const fn active(self) -> bool {
        !matches!(
            self.phase,
            ProcedurePhase::Complete | ProcedurePhase::Failed
        )
    }

    pub const fn response_deadline_us(&self) -> Option<u64> {
        if self.response_deadline_us == 0 {
            None
        } else {
            Some(self.response_deadline_us)
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProcedureError {
    Busy,
    Collision(u8),
    MissingContext,
    StaleToken,
    InvalidPhase,
    WrongOpcode,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProcedureManager {
    local: Option<ProcedureContext>,
    remote: Option<ProcedureContext>,
}

const _: () = {
    assert!(core::mem::size_of::<ProcedureContext>() <= 48);
    assert!(core::mem::size_of::<ProcedureManager>() <= 96);
};

impl ProcedureManager {
    pub const fn new() -> Self {
        Self {
            local: None,
            remote: None,
        }
    }

    pub const fn local(&self) -> Option<&ProcedureContext> {
        self.local.as_ref()
    }

    pub const fn remote(&self) -> Option<&ProcedureContext> {
        self.remote.as_ref()
    }

    pub fn start_local(&mut self, context: ProcedureContext) -> Result<(), ProcedureError> {
        if context.initiator != ProcedureInitiator::Local {
            return Err(ProcedureError::InvalidPhase);
        }
        if self.local.is_some_and(ProcedureContext::active) {
            return Err(ProcedureError::Busy);
        }
        self.local = Some(context);
        Ok(())
    }

    pub fn start_remote(&mut self, mut context: ProcedureContext) -> Result<(), ProcedureError> {
        if context.initiator != ProcedureInitiator::Remote {
            return Err(ProcedureError::InvalidPhase);
        }
        if let Some(remote) = self.remote.filter(|remote| remote.active()) {
            return Err(Self::collision(remote.kind, context.kind));
        }
        if let Some(local) = self.local.filter(|local| local.active())
            && local.kind.instant_procedure().is_some()
            && context.kind.instant_procedure().is_some()
        {
            return Err(Self::collision(local.kind, context.kind));
        }
        // Remote instant-bearing indications are already on air and need no
        // local TX acknowledgement before their instant can be observed.
        if context.instant.is_some() {
            context.phase = ProcedurePhase::AwaitingInstant;
        }
        self.remote = Some(context);
        Ok(())
    }

    fn collision(local: ProcedureKind, remote: ProcedureKind) -> ProcedureError {
        match (local.instant_procedure(), remote.instant_procedure()) {
            (Some(local), Some(remote)) => {
                ProcedureError::Collision(procedure_collision_error(local, remote))
            }
            _ => ProcedureError::Busy,
        }
    }

    fn context_mut(
        &mut self,
        initiator: ProcedureInitiator,
    ) -> Result<&mut ProcedureContext, ProcedureError> {
        match initiator {
            ProcedureInitiator::Local => self.local.as_mut(),
            ProcedureInitiator::Remote => self.remote.as_mut(),
        }
        .ok_or(ProcedureError::MissingContext)
    }

    /// Planning is deliberately validation-only. A queued PDU remains queued
    /// until the exact physical transmission completes.
    pub fn tx_planned(
        &mut self,
        initiator: ProcedureInitiator,
        token: ProcedureToken,
    ) -> Result<(), ProcedureError> {
        let context = self.context_mut(initiator)?;
        if context.token != token {
            return Err(ProcedureError::StaleToken);
        }
        if !matches!(
            context.phase,
            ProcedurePhase::Queued | ProcedurePhase::ResponseQueued
        ) {
            return Err(ProcedureError::InvalidPhase);
        }
        Ok(())
    }

    pub fn tx_done(
        &mut self,
        initiator: ProcedureInitiator,
        token: ProcedureToken,
    ) -> Result<(), ProcedureError> {
        let context = self.context_mut(initiator)?;
        if context.token != token {
            return Err(ProcedureError::StaleToken);
        }
        if !matches!(
            context.phase,
            ProcedurePhase::Queued | ProcedurePhase::ResponseQueued
        ) {
            return Err(ProcedureError::InvalidPhase);
        }
        context.phase = ProcedurePhase::AwaitingTxAck;
        Ok(())
    }

    pub fn nesn_ack(
        &mut self,
        initiator: ProcedureInitiator,
        token: ProcedureToken,
        now_us: u64,
    ) -> Result<(), ProcedureError> {
        let context = self.context_mut(initiator)?;
        if context.token != token {
            return Err(ProcedureError::StaleToken);
        }
        if context.phase != ProcedurePhase::AwaitingTxAck {
            return Err(ProcedureError::InvalidPhase);
        }
        if context.pending_response {
            context.phase = if context.instant.is_some() {
                ProcedurePhase::AwaitingInstant
            } else if context.expected_response_opcode.is_some() {
                context.response_deadline_us = if context.response_timeout_us == 0 {
                    0
                } else {
                    now_us.saturating_add(u64::from(context.response_timeout_us))
                };
                ProcedurePhase::AwaitingResponse
            } else {
                ProcedurePhase::Complete
            };
            context.pending_response = false;
        } else if context.expected_response_opcode.is_some() {
            context.phase = ProcedurePhase::AwaitingResponse;
            context.response_deadline_us = if context.response_timeout_us == 0 {
                0
            } else {
                now_us.saturating_add(u64::from(context.response_timeout_us))
            };
        } else if context.instant.is_some() {
            context.phase = ProcedurePhase::AwaitingInstant;
        } else {
            context.phase = ProcedurePhase::Complete;
        }
        Ok(())
    }

    pub fn expected_response(
        &mut self,
        initiator: ProcedureInitiator,
        opcode: u8,
    ) -> Result<(), ProcedureError> {
        let context = self.context_mut(initiator)?;
        if context.phase != ProcedurePhase::AwaitingResponse {
            return Err(ProcedureError::InvalidPhase);
        }
        if context.expected_response_opcode != Some(opcode) {
            return Err(ProcedureError::WrongOpcode);
        }
        context.response_deadline_us = 0;
        context.phase = if context.instant.is_some() {
            ProcedurePhase::AwaitingInstant
        } else {
            ProcedurePhase::Complete
        };
        Ok(())
    }

    /// Accept an expected indication whose procedure completes only at its
    /// instant. The procedure response timer remains armed until that instant
    /// is applied, matching the connection-parameter request sequence used by
    /// a Peripheral (`REQ` -> `UPDATE_IND` -> instant).
    pub fn expected_response_with_instant(
        &mut self,
        initiator: ProcedureInitiator,
        opcode: u8,
        instant: u16,
    ) -> Result<(), ProcedureError> {
        let context = self.context_mut(initiator)?;
        if context.phase != ProcedurePhase::AwaitingResponse {
            return Err(ProcedureError::InvalidPhase);
        }
        if context.expected_response_opcode != Some(opcode) {
            return Err(ProcedureError::WrongOpcode);
        }
        context.instant = Some(instant);
        context.phase = ProcedurePhase::AwaitingInstant;
        Ok(())
    }

    /// Mark that the local response to a remote procedure now owns the exact
    /// token. Enqueueing it does not prove TX or restart an unrelated timer.
    pub fn response_queued(
        &mut self,
        token: ProcedureToken,
        tx_opcode: u8,
    ) -> Result<(), ProcedureError> {
        let context = self.context_mut(ProcedureInitiator::Remote)?;
        if !matches!(
            context.phase,
            ProcedurePhase::Queued | ProcedurePhase::AwaitingResponse
        ) {
            return Err(ProcedureError::InvalidPhase);
        }
        context.token = token;
        context.tx_opcode = tx_opcode;
        context.pending_response = true;
        context.phase = ProcedurePhase::ResponseQueued;
        Ok(())
    }

    pub fn check_timeouts(&mut self, now_us: u64, error: u8) -> bool {
        let mut failed = false;
        for context in [&mut self.local, &mut self.remote].into_iter().flatten() {
            if matches!(
                context.phase,
                ProcedurePhase::AwaitingResponse | ProcedurePhase::AwaitingInstant
            ) && context.response_deadline_us != 0
                && now_us >= context.response_deadline_us
            {
                context.phase = ProcedurePhase::Failed;
                context.error = Some(error);
                context.response_deadline_us = 0;
                failed = true;
            }
        }
        failed
    }

    pub fn fail(&mut self, initiator: ProcedureInitiator, error: u8) -> Result<(), ProcedureError> {
        let context = self.context_mut(initiator)?;
        context.phase = ProcedurePhase::Failed;
        context.error = Some(error);
        context.response_deadline_us = 0;
        context.pending_response = false;
        Ok(())
    }

    pub fn on_connection_event(&mut self, event_counter: u16, passed_error: u8) {
        for context in [&mut self.local, &mut self.remote].into_iter().flatten() {
            if context.phase != ProcedurePhase::AwaitingInstant {
                continue;
            }
            let Some(instant) = context.instant else {
                continue;
            };
            match classify_instant(event_counter, instant) {
                InstantRelation::Current => {
                    context.phase = ProcedurePhase::Complete;
                    context.response_deadline_us = 0;
                }
                InstantRelation::Future(_) => {}
                InstantRelation::Passed => {
                    context.phase = ProcedurePhase::Failed;
                    context.error = Some(passed_error);
                    context.response_deadline_us = 0;
                }
            }
        }
    }
}

impl Default for ProcedureManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOKEN: ProcedureToken = ProcedureToken {
        owner_token: 0x1234,
        generation: 7,
    };

    fn local_request(kind: ProcedureKind, response: Option<u8>) -> ProcedureContext {
        ProcedureContext::new(
            kind,
            ProcedureInitiator::Local,
            TOKEN,
            0x0f,
            response,
            None,
            Some(40_000_000),
        )
    }

    #[test]
    fn queue_plan_tx_ack_and_expected_response_are_distinct() {
        let mut manager = ProcedureManager::new();
        manager
            .start_local(local_request(
                ProcedureKind::ConnectionParameters,
                Some(0x10),
            ))
            .unwrap();
        manager
            .tx_planned(ProcedureInitiator::Local, TOKEN)
            .unwrap();
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Queued);
        manager.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingTxAck
        );
        manager
            .nesn_ack(ProcedureInitiator::Local, TOKEN, 5_000)
            .unwrap();
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingResponse
        );
        assert_eq!(
            manager.local().unwrap().response_deadline_us(),
            Some(40_005_000)
        );
        assert_eq!(
            manager.expected_response(ProcedureInitiator::Local, 0x11),
            Err(ProcedureError::WrongOpcode)
        );
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingResponse
        );
        manager
            .expected_response(ProcedureInitiator::Local, 0x10)
            .unwrap();
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Complete);
        assert_eq!(manager.local().unwrap().response_deadline_us(), None);
    }

    #[test]
    fn stale_generation_and_duplicate_completion_cannot_advance() {
        let mut manager = ProcedureManager::new();
        manager
            .start_local(local_request(ProcedureKind::Ping, Some(0x13)))
            .unwrap();
        let stale = ProcedureToken {
            generation: TOKEN.generation.wrapping_add(1),
            ..TOKEN
        };
        assert_eq!(
            manager.tx_done(ProcedureInitiator::Local, stale),
            Err(ProcedureError::StaleToken)
        );
        manager.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        assert_eq!(
            manager.tx_done(ProcedureInitiator::Local, TOKEN),
            Err(ProcedureError::InvalidPhase)
        );
    }

    #[test]
    fn local_termination_requires_exact_tx_and_ack() {
        let mut manager = ProcedureManager::new();
        let terminate = ProcedureContext::new(
            ProcedureKind::Termination,
            ProcedureInitiator::Local,
            TOKEN,
            0x02,
            None,
            None,
            None,
        );
        manager.start_local(terminate).unwrap();
        manager
            .tx_planned(ProcedureInitiator::Local, TOKEN)
            .unwrap();
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Queued);
        manager.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingTxAck
        );
        manager
            .nesn_ack(ProcedureInitiator::Local, TOKEN, 0)
            .unwrap();
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Complete);
        assert_eq!(manager.local().unwrap().response_deadline_us(), None);
    }

    #[test]
    fn unrelated_queue_activity_cannot_restart_response_deadline() {
        let mut manager = ProcedureManager::new();
        manager
            .start_local(local_request(ProcedureKind::Ping, Some(0x13)))
            .unwrap();
        manager.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        manager
            .nesn_ack(ProcedureInitiator::Local, TOKEN, 100)
            .unwrap();
        let deadline = manager.local().unwrap().response_deadline_us();
        assert_eq!(
            manager.start_local(local_request(ProcedureKind::FeatureExchange, Some(0x09))),
            Err(ProcedureError::Busy)
        );
        assert_eq!(manager.local().unwrap().response_deadline_us(), deadline);
        assert!(!manager.check_timeouts(deadline.unwrap() - 1, 0x22));
        assert!(manager.check_timeouts(deadline.unwrap(), 0x22));
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Failed);
        assert_eq!(manager.local().unwrap().error, Some(0x22));
    }

    #[test]
    fn same_and_different_instant_collisions_keep_existing_owner() {
        let mut manager = ProcedureManager::new();
        manager
            .start_local(local_request(
                ProcedureKind::ConnectionParameters,
                Some(0x10),
            ))
            .unwrap();
        let remote_same = ProcedureContext::new(
            ProcedureKind::ConnectionUpdate,
            ProcedureInitiator::Remote,
            TOKEN,
            0x00,
            None,
            Some(200),
            None,
        );
        assert_eq!(
            manager.start_remote(remote_same),
            Err(ProcedureError::Collision(0x23))
        );
        let remote_different = ProcedureContext {
            kind: ProcedureKind::ChannelMapUpdate,
            ..remote_same
        };
        assert_eq!(
            manager.start_remote(remote_different),
            Err(ProcedureError::Collision(0x2a))
        );
        assert_eq!(manager.remote(), None);
        assert_eq!(
            manager.local().unwrap().kind,
            ProcedureKind::ConnectionParameters
        );
    }

    #[test]
    fn instant_applies_only_at_current_event_including_wraparound() {
        let mut manager = ProcedureManager::new();
        let context = ProcedureContext::new(
            ProcedureKind::ChannelMapUpdate,
            ProcedureInitiator::Local,
            TOKEN,
            0x01,
            None,
            Some(1),
            None,
        );
        manager.start_local(context).unwrap();
        manager.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        manager
            .nesn_ack(ProcedureInitiator::Local, TOKEN, 0)
            .unwrap();
        manager.on_connection_event(0xfffe, 0x28);
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
        manager.on_connection_event(1, 0x28);
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Complete);
    }

    #[test]
    fn remote_response_requires_queue_tx_done_and_nesn_ack() {
        let mut manager = ProcedureManager::new();
        let request = ProcedureContext::new(
            ProcedureKind::FeatureExchange,
            ProcedureInitiator::Remote,
            TOKEN,
            0,
            None,
            None,
            None,
        );
        manager.start_remote(request).unwrap();
        manager.response_queued(TOKEN, 0x09).unwrap();
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::ResponseQueued
        );
        manager.tx_done(ProcedureInitiator::Remote, TOKEN).unwrap();
        manager
            .nesn_ack(ProcedureInitiator::Remote, TOKEN, 0)
            .unwrap();
        assert_eq!(manager.remote().unwrap().phase, ProcedurePhase::Complete);
    }

    #[test]
    fn remote_parameter_response_ack_starts_followup_deadline_until_instant() {
        let mut manager = ProcedureManager::new();
        let context = ProcedureContext::new(
            ProcedureKind::ConnectionParameters,
            ProcedureInitiator::Remote,
            TOKEN,
            0x10,
            Some(0x00),
            None,
            Some(40_000_000),
        );
        manager.start_remote(context).unwrap();
        assert_eq!(manager.remote().unwrap().phase, ProcedurePhase::Queued);
        assert_eq!(manager.remote().unwrap().response_deadline_us(), None);
        manager.response_queued(TOKEN, 0x10).unwrap();
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::ResponseQueued
        );
        assert_eq!(manager.remote().unwrap().response_deadline_us(), None);
        manager.tx_done(ProcedureInitiator::Remote, TOKEN).unwrap();
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::AwaitingTxAck
        );
        assert_eq!(manager.remote().unwrap().response_deadline_us(), None);
        manager
            .nesn_ack(ProcedureInitiator::Remote, TOKEN, 2_000)
            .unwrap();
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::AwaitingResponse
        );
        assert_eq!(
            manager.remote().unwrap().response_deadline_us(),
            Some(40_002_000)
        );
        manager
            .expected_response_with_instant(ProcedureInitiator::Remote, 0x00, 30)
            .unwrap();
        manager.on_connection_event(29, 0x28);
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
        manager.on_connection_event(30, 0x28);
        assert_eq!(manager.remote().unwrap().phase, ProcedurePhase::Complete);
        assert_eq!(manager.remote().unwrap().response_deadline_us(), None);
    }

    #[test]
    fn remote_instant_indication_does_not_wait_for_local_tx() {
        let mut manager = ProcedureManager::new();
        let indication = ProcedureContext::new(
            ProcedureKind::ChannelMapUpdate,
            ProcedureInitiator::Remote,
            TOKEN,
            0x01,
            None,
            Some(10),
            None,
        );
        manager.start_remote(indication).unwrap();
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
        manager.on_connection_event(9, 0x28);
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
        let competing = ProcedureContext {
            kind: ProcedureKind::ConnectionUpdate,
            tx_opcode: 0x00,
            instant: Some(11),
            ..indication
        };
        assert_eq!(
            manager.start_remote(competing),
            Err(ProcedureError::Collision(0x2a))
        );
        manager.on_connection_event(10, 0x28);
        assert_eq!(manager.remote().unwrap().phase, ProcedurePhase::Complete);
        manager.start_remote(competing).unwrap();
        assert_eq!(
            manager.remote().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
    }

    #[test]
    fn fixed_two_context_manager_stays_within_compact_budget() {
        assert!(core::mem::size_of::<ProcedureContext>() <= 48);
        assert!(core::mem::size_of::<ProcedureManager>() <= 96);
    }

    #[test]
    fn failed_queue_attempt_releases_the_remote_context() {
        let mut manager = ProcedureManager::new();
        let remote = ProcedureContext::new(
            ProcedureKind::Ping,
            ProcedureInitiator::Remote,
            TOKEN,
            0x13,
            None,
            None,
            None,
        );
        manager.start_remote(remote).unwrap();
        manager.fail(ProcedureInitiator::Remote, 0x3a).unwrap();
        assert_eq!(manager.remote().unwrap().phase, ProcedurePhase::Failed);
        assert_eq!(manager.remote().unwrap().error, Some(0x3a));
        manager.start_remote(remote).unwrap();
    }

    #[test]
    fn peripheral_connection_parameters_keep_timeout_until_update_instant() {
        let mut manager = ProcedureManager::new();
        let request = ProcedureContext::new(
            ProcedureKind::ConnectionParameters,
            ProcedureInitiator::Local,
            TOKEN,
            0x0f,
            Some(0x00),
            None,
            Some(40_000_000),
        );
        manager.start_local(request).unwrap();
        manager.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        manager
            .nesn_ack(ProcedureInitiator::Local, TOKEN, 1_000)
            .unwrap();
        manager
            .expected_response_with_instant(ProcedureInitiator::Local, 0x00, 25)
            .unwrap();
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
        assert_eq!(
            manager.local().unwrap().response_deadline_us(),
            Some(40_001_000)
        );
        manager.on_connection_event(24, 0x28);
        assert_eq!(
            manager.local().unwrap().phase,
            ProcedurePhase::AwaitingInstant
        );
        manager.on_connection_event(25, 0x28);
        assert_eq!(manager.local().unwrap().phase, ProcedurePhase::Complete);
        assert_eq!(manager.local().unwrap().response_deadline_us(), None);

        let mut timed_out = ProcedureManager::new();
        timed_out.start_local(request).unwrap();
        timed_out.tx_done(ProcedureInitiator::Local, TOKEN).unwrap();
        timed_out
            .nesn_ack(ProcedureInitiator::Local, TOKEN, 1_000)
            .unwrap();
        timed_out
            .expected_response_with_instant(ProcedureInitiator::Local, 0x00, 25)
            .unwrap();
        assert!(!timed_out.check_timeouts(40_000_999, 0x22));
        assert!(timed_out.check_timeouts(40_001_000, 0x22));
        assert_eq!(timed_out.local().unwrap().phase, ProcedurePhase::Failed);
        assert_eq!(timed_out.local().unwrap().error, Some(0x22));
        assert_eq!(timed_out.local().unwrap().response_deadline_us(), None);
    }
}

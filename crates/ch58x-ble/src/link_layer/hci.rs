//! HCI commands and events; RF scheduling remains in the parent runner.
use super::*;

impl Runner {
    pub(super) async fn handle_command(&mut self, bytes: &[u8]) {
        if bytes.len() < 3 || bytes.len() != 3 + usize::from(bytes[2]) {
            return;
        }
        let opcode = u16::from_le_bytes([bytes[0], bytes[1]]);
        let params = &bytes[3..];
        let result: Result<&[u8], u8> = match opcode {
            0x0c03 if params.is_empty() => {
                controller::set_event_mask(u64::MAX);
                controller::set_le_event_mask(0x1f);
                self.advertiser.enabled = false;

                self.radio.set_access_address(ADV_ACCESS_ADDRESS);
                self.radio.set_crc_init(ADV_CRC_INIT);
                Ok(&[][..])
            }
            0x0c01 | 0x2001 if params.len() == 8 => {
                let mask = u64::from_le_bytes(params.try_into().unwrap());
                if opcode == 0x0c01 {
                    controller::set_event_mask(mask);
                } else {
                    controller::set_le_event_mask(mask);
                }
                Ok(&[][..])
            }
            // This in-process transport supports no Controller-to-Host flow
            // control. A Host may explicitly disable it.
            0x0c31 if params == [0] => Ok(&[][..]),
            0x1009 if params.is_empty() => Ok(&[0, 0, 0, 0, 0, 0][..]),
            // Credits are returned only after the peer acknowledges an ACL owner.
            0x2002 if params.is_empty() => {
                const BUFFER_SIZE: [u8; 3] = [0x1b, 0x00, ble_controller::hci::ACL_CREDITS];
                Ok(&BUFFER_SIZE)
            }
            // Connection Parameters Request procedure, and
            // Extended Reject Indication.
            0x2003 if params.is_empty() => Ok(&ble_controller::hci::LE_FEATURES),
            0x2005 => self.set_random_address(params),
            0x2006 => self.set_advertising_parameters(params),
            0x2008 => Self::set_advertising_data(&mut self.advertiser.data, params),
            0x2009 => Self::set_advertising_data(&mut self.advertiser.scan_response, params),
            0x200a => self.set_advertising_enable(params),

            // One software filter-list entry is added with initiation. Until
            // then report zero capacity instead of accepting unusable state.
            0x200f if params.is_empty() => Ok(&[0][..]),
            0x2010 if params.is_empty() => Ok(&[][..]),
            // Bytes from the caller-seeded PRNG; this is not a hardware entropy source.
            0x2018 if params.is_empty() => {
                let mut random = [0u8; 8];
                self.random.fill_bytes(&mut random);
                controller::command_complete(opcode, HCI_SUCCESS, &random).await;
                return;
            }
            // Extended advertising is unsupported.
            0x2039 => Err(HCI_COMMAND_DISALLOWED),

            0x200d => {
                controller::command_status(opcode, HCI_COMMAND_DISALLOWED).await;
                return;
            }
            // Other async procedures receive Command Status; completion comes
            // from the associated Link Layer event.
            0x2013 | 0x2019 | 0x2032 => {
                controller::command_status(opcode, HCI_COMMAND_DISALLOWED).await;
                return;
            }
            _ => Err(HCI_UNKNOWN_COMMAND),
        };

        match result {
            Ok(return_data) => controller::command_complete(opcode, HCI_SUCCESS, return_data).await,
            Err(status) => controller::command_complete(opcode, status, &[]).await,
        }
    }

    fn set_random_address(&mut self, params: &[u8]) -> Result<&'static [u8], u8> {
        if self.advertiser.enabled {
            return Err(HCI_COMMAND_DISALLOWED);
        }
        let bytes: [u8; 6] = params.try_into().map_err(|_| HCI_INVALID_PARAMETERS)?;
        self.advertiser.address =
            DeviceAddress::random_static(bytes).map_err(|_| HCI_INVALID_PARAMETERS)?;
        Ok(&[])
    }

    fn set_advertising_parameters(&mut self, params: &[u8]) -> Result<&'static [u8], u8> {
        if self.advertiser.enabled {
            return Err(HCI_COMMAND_DISALLOWED);
        }
        let parsed = ble_controller::hci::AdvertisingParameters::parse(params)
            .ok_or(HCI_INVALID_PARAMETERS)?;
        self.advertiser.interval = Duration::from_micros(parsed.interval_us);
        self.advertiser.channels = parsed.channels;
        Ok(&[])
    }

    fn set_advertising_data(
        target: &mut AdvertisingData,
        params: &[u8],
    ) -> Result<&'static [u8], u8> {
        if params.len() != 32 || usize::from(params[0]) > 31 {
            return Err(HCI_INVALID_PARAMETERS);
        }
        let mut replacement = AdvertisingData::new();
        let bytes = &params[1..1 + usize::from(params[0])];
        // Preserve already-encoded AD structures without interpreting them.
        let mut cursor = 0;
        while cursor < bytes.len() {
            let len = usize::from(bytes[cursor]);
            if len == 0 {
                break;
            }
            if cursor + len >= bytes.len() || len < 1 {
                return Err(HCI_INVALID_PARAMETERS);
            }
            replacement
                .push(bytes[cursor + 1], &bytes[cursor + 2..cursor + 1 + len])
                .map_err(|_| HCI_INVALID_PARAMETERS)?;
            cursor += len + 1;
        }
        *target = replacement;
        Ok(&[])
    }

    fn set_advertising_enable(&mut self, params: &[u8]) -> Result<&'static [u8], u8> {
        if params.len() != 1 || params[0] > 1 {
            return Err(HCI_INVALID_PARAMETERS);
        }
        if params[0] != 0 && is_connected() {
            return Err(HCI_COMMAND_DISALLOWED);
        }
        self.advertiser.enabled = params[0] != 0;

        Ok(&[])
    }

    pub(super) async fn emit_connection_complete(&self, connection: &ActiveConnection) {
        let mut params = [0u8; 19];
        params[0] = 0x01;
        params[1] = HCI_SUCCESS;
        params[2..4].copy_from_slice(&connection.handle.to_le_bytes());
        params[4] = 0x01; // Peripheral role.
        params[5] = u8::from(connection.request.initiator_address_random);
        params[6..12].copy_from_slice(&connection.request.initiator_address);
        params[12..14].copy_from_slice(&connection.request.interval.to_le_bytes());
        params[14..16].copy_from_slice(&connection.request.latency.to_le_bytes());
        params[16..18].copy_from_slice(&connection.request.supervision_timeout.to_le_bytes());
        params[18] = connection.request.sleep_clock_accuracy;
        let _ = controller::event(0x3e, &params).await;
    }

    pub(super) async fn emit_disconnection(&self, connection: &ActiveConnection, reason: u8) {
        let mut params = [0u8; 4];
        params[0] = HCI_SUCCESS;
        params[1..3].copy_from_slice(&connection.handle.to_le_bytes());
        params[3] = reason;
        let _ = controller::event(0x05, &params).await;
    }

    pub(super) async fn emit_connection_update_complete(&self, connection: &ActiveConnection) {
        self.emit_connection_update_result(connection, HCI_SUCCESS)
            .await;
    }

    pub(super) async fn emit_connection_update_result(
        &self,
        connection: &ActiveConnection,
        status: u8,
    ) {
        let mut params = [0u8; 10];
        params[0] = 0x03;
        params[1] = status;
        params[2..4].copy_from_slice(&connection.handle.to_le_bytes());
        params[4..6].copy_from_slice(&connection.request.interval.to_le_bytes());
        params[6..8].copy_from_slice(&connection.request.latency.to_le_bytes());
        params[8..10].copy_from_slice(&connection.request.supervision_timeout.to_le_bytes());
        let _ = controller::event(0x3e, &params).await;
    }

    /// Returns true only when the connection must stop immediately.
    pub(super) async fn handle_connected_command(
        &mut self,
        connection: &mut ActiveConnection,
        bytes: &[u8],
    ) -> bool {
        if bytes.len() < 3 || bytes.len() != 3 + usize::from(bytes[2]) {
            return false;
        }
        let opcode = u16::from_le_bytes([bytes[0], bytes[1]]);
        let params = &bytes[3..];
        match opcode {
            0x0c03 if params.is_empty() => {
                self.handle_command(bytes).await;
                true
            }
            0x0406 if params.len() == 3 => {
                let handle = u16::from_le_bytes([params[0], params[1]]);
                if handle != connection.handle
                    || connection.local_procedure_active(ProcedureKind::Termination)
                {
                    controller::command_status(opcode, HCI_COMMAND_DISALLOWED).await;
                    return false;
                }
                if connection.queue_local_procedure(
                    ProcedureKind::Termination,
                    &[0x02, params[2]],
                    None,
                    None,
                ) {
                    controller::command_status(opcode, HCI_SUCCESS).await;
                } else {
                    controller::command_status(opcode, HCI_CONTROLLER_BUSY).await;
                }
                false
            }
            0x1405 if params.len() == 2 => {
                if u16::from_le_bytes([params[0], params[1]]) != connection.handle {
                    controller::command_complete(opcode, 0x02, params).await;
                    return false;
                }
                let mut ret = [0u8; 3];
                ret[..2].copy_from_slice(&connection.handle.to_le_bytes());
                ret[2] = connection.last_rssi as u8;
                controller::command_complete(opcode, HCI_SUCCESS, &ret).await;
                false
            }
            0x2013 if params.len() == 14 => {
                let handle = u16::from_le_bytes([params[0], params[1]]);
                let interval_min = u16::from_le_bytes([params[2], params[3]]);
                let interval_max = u16::from_le_bytes([params[4], params[5]]);
                let latency = u16::from_le_bytes([params[6], params[7]]);
                let timeout = u16::from_le_bytes([params[8], params[9]]);
                if handle != connection.handle
                    || !connection_parameter_range_is_valid(
                        interval_min,
                        interval_max,
                        latency,
                        timeout,
                    )
                {
                    controller::command_status(opcode, HCI_INVALID_PARAMETERS).await;
                    return false;
                }

                if connection.local_connection_param_request.is_some()
                    || connection.remote_connection_parameters_active()
                    || connection.pending_connection_update.is_some()
                    || connection.pending_channel_map.is_some()
                {
                    controller::command_status(opcode, HCI_CONTROLLER_BUSY).await;
                    return false;
                }

                let request = ConnectionParamPdu {
                    interval_min,
                    interval_max,
                    latency,
                    supervision_timeout: timeout,
                    preferred_periodicity: 0,
                    reference_event_count: connection.event_counter,
                    offsets: [u16::MAX; 6],
                };
                let mut pdu = [0u8; ConnectionParamPdu::PDU_LEN];
                request.write(ConnectionParamPdu::REQUEST_OPCODE, &mut pdu);
                // A Peripheral waits for the Central's
                // LL_CONNECTION_UPDATE_IND, not LL_CONNECTION_PARAM_RSP.
                if !connection.queue_local_procedure_request(
                    ProcedureKind::ConnectionParameters,
                    &pdu,
                    0x00,
                ) {
                    controller::command_status(opcode, HCI_CONTROLLER_BUSY).await;
                    return false;
                }
                connection.local_connection_param_request = Some(LocalConnectionParamRequest {
                    interval_min,
                    interval_max,
                    latency,
                    supervision_timeout: timeout,
                });
                controller::command_status(opcode, HCI_SUCCESS).await;
                false
            }
            _ => {
                self.handle_command(bytes).await;
                false
            }
        }
    }
}

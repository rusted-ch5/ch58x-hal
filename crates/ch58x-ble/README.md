# CH585 backend

Experimental plaintext BLE Peripheral backend. `controller` is the bounded HCI
transport; `link_layer` owns scheduling and LL procedures; `link_layer/hci` maps
commands/events; `radio` owns DMA and BLEB; `rf_frontend` calibrates the analog RF
block. The crate consumes the root HAL's unique BLE token.

This is CH585-only and Git-only. Read `docs/ble-integration.md` and
`docs/ble-status.md` at the repository root before integration. Host tests and
cross-compilation do not prove this extracted firmware works on air.

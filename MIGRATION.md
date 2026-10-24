# BLE extraction provenance

2026-10-24. Public base: `051d966243e3bfa6a62529e9039ea979fec78d3a`.
Source snapshot: `ch582mdev` commit `8a0b6e0` (source-only extraction;
no product Git history imported). Existing HAL licenses remain unchanged.

Allowlist:

- `ble-controller/src/{advertising,channel,crc,phy,rf_calibration,whitening,link_layer,llcp}.rs`
- `ch58x-ble/src/{controller,link_layer,radio,radio_dma_policy,radio_rx_status,rf_frontend}.rs`

The feature selection starts with CH585 and the compact HCI send future.
Disabled features and their instrumentation are removed. AES/CCM, passkeys,
security state, Central initiation, dual-role scheduling, product diagnostics,
performance experiments, application logic, and Nordic fixture source/binaries
are excluded. This is a reviewed adaptation, not a byte-identical copy.

Explicit adaptations after selection:

- HAL `BLE` token replaces private runtime ownership; the core has no HAL
  dependency. Backend dependencies are public packages and a pinned public runtime.
- Encryption handlers/cache/state are removed together with the advertised LE
  encryption feature. The backend accepts plaintext packets of at most 27 bytes.
- HCI command/event handling lives in `link_layer/hci.rs`; queue serialization
  remains in `controller.rs`; scheduling/owner state remains in `link_layer.rs`.
- CH585-only single-exchange radio operation excludes conditional crypto follow-RX
  and experimental multi-PDU paths. RX/TX waits are bounded.
- Advertising observes the capture flag before reading fresh completion status;
  DMA reads are volatile. This ordering follows the independently tested C fix,
  but does not inherit that C image's hardware evidence.
- A peer PDU received before the current TX cannot acknowledge a newly staged
  owner. The portable outstanding-owner rule is used for this boundary.
- HCI event masks are enforced, advertising parameters are validated before state
  mutation, unsupported capabilities are rejected, and truncated receive buffers
  are rejected. PRNG output is documented as caller-seeded, not measured entropy.
- Register-independent DMA/timing/status logic has a standalone host-test case.

No private repository remote, user credential, binary vendor library, generated
firmware, or Nordic fixture implementation is needed to build these libraries.
The optional Embassy integration needs a single patched HAL source, described in
`docs/ble-integration.md`. No crates.io release or Bluetooth qualification is claimed.

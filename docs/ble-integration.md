# Integrating the experimental CH585 backend

The ownership chain is:

```text
HAL Peripherals.BLE
        | split
        +-- ControllerToken -> controller::new / HciTransport -> Bluetooth Host
        +-- RadioToken      -> link_layer::Runner              -> radio / RFEND
```

Initialize CH585 clocks and a working 1-MHz Embassy time driver first. Supply
`Runner::new` with the radio token and an application-seeded `ChaCha20Rng`, then
poll `Runner::run` concurrently with the Host. Consume the controller token only
once. `Runner::new` accesses MMIO and calibrates RF; it is not a host-test constructor.
The backend owns BLEB/BLEL and the RF/DMA state. Do not bind a second BLE handler.

`ch58x-ble-controller` contains protocol primitives and procedure ownership. It
has no register access, HCI queue, GATT database, pairing, or keyboard behavior.
`ch58x-ble::controller` provides bounded `bt-hci` transport queues. A caller must
drain events and ACL data while the runner is active; back-pressure can suspend
the runner, so a Host that stops reading cannot sustain RF deadlines.
`ch58x-ble::link_layer::hci` maps commands/events; the parent runner schedules RF
and owns the connection. `radio` and `rf_frontend` contain CH585 register code.

Current scope: legacy connectable advertising and scan responses, one plaintext
Peripheral link, 27-byte ACL payloads/five credits, CSA#1, LL control procedures,
connection/channel-map instants, supervision, termination, and bounded HCI queues.
There is no bundled ATT/GATT Host. Central scanning/initiation, security, DLE,
extended advertising, PHY updates and a complete general-purpose HCI command set
are not release capabilities. The portable core has more reusable sequence
primitives than this backend exposes; those do not imply multi-PDU RF support.

The pinned public `embassy-ch58x` currently uses the older HAL revision. An
application must unify its HAL source with this workspace before consuming the
new BLE token. For a local checkout, add to the **application's root manifest**:

```toml
[patch."https://github.com/rusted-ch5/ch58x-hal.git"]
ch58x-hal-rs = { path = "../ch58x-hal" }
```

The application can depend on `crates/ch58x-ble` by path during review. For Git
consumption, pin the HAL and BLE packages to the same reviewed commit and patch
the Embassy HAL dependency to that source. Do not construct a second singleton
or use `BLE::steal` to hide duplicate HAL versions. The BLE library itself needs
no private `embassy-ch58x` source and does not install a time driver.

Run `cases/check-ble.sh` for the portable and library checks. It never flashes a
board. Firmware linking, stack/RAM measurement and on-air acceptance are separate
steps described in `ble-peer-contract.md`.

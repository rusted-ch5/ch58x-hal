# Portable BLE controller core

Package `ch58x-ble-controller`, Rust import `ble_controller`.
No MMIO, allocator, vendor runtime, Host stack or security engine. Modules cover
legacy advertising parsing, PHY transforms, channel selection, packet ownership,
LLCP context/instants, RF calibration arithmetic and HCI validation. Protocol
helpers do not themselves implement a controller or establish RF acceptance.

Run `cargo test -p ch58x-ble-controller --target <host-triple>` from the workspace.
See the repository `docs/ble-status.md` for scope and evidence.

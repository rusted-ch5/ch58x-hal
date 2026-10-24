# ch58x-hal

Pure Rust hardware abstraction for WCH CH58x microcontrollers.

The crates.io package is `ch58x-hal-rs`; its Rust library name remains
`ch58x_hal`.

The crate currently provides:

- compile-time selection for CH582 and CH585;
- PAC access through `ch58x_hal::pac`;
- single-owner peripheral tokens;
- a QingKe-compatible global critical-section implementation;
- system-clock setup for CH582 and CH585;
- owned digital input/output pins implementing `embedded-hal` 1.0 traits;
- bounded blocking ADC conversions, including temperature-sensor support;
- coherent RTC counter and day reads;
- an asynchronous NOR-flash trait implementation for the CH58x DataFlash
  window;
- blocking UART0 through UART3 with checked baud-rate selection and
  `embedded-io` traits plus non-blocking FIFO access.

Enable the `rt` feature when linking a complete firmware image with
`qingke-rt`.

## Status

The crate is under active development. APIs may change before the first stable
release.

## License

Licensed under either of
[Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at
your option.

## Workspace modules

| Package / path | Responsibility | Dependencies |
| --- | --- | --- |
| `ch58x-hal-rs` / repository root | CH582/CH585 clocks, GPIO, UART, DataFlash, peripheral ownership including `BLE` | PAC and embedded traits; no BLE stack |
| `ch58x-ble-controller` / `crates/ble-controller` | Portable advertising parser, channel selection, SN/NESN ownership, LLCP instants/timeouts and HCI validation | `no_std`, Embassy time types; no MMIO or Host stack |
| `ch58x-ble` / `crates/ch58x-ble` | Experimental CH585 RF calibration, DMA/IRQ, single-connection Peripheral runner and bounded HCI transport | HAL + portable core + Embassy primitives |

The default workspace member is still the HAL. Select a BLE package explicitly;
`--workspace` would combine the default CH582 HAL with the CH585-only backend.
The HAL retains its Rust 1.85 contract. The BLE crates require Rust 1.88 or newer.

The BLE packages are Git-only development snapshots (`publish = false`). They
contain no encryption engine, security manager, keyboard application, or Nordic
fixture implementation. This snapshot has host-test and library-build evidence,
**not Rust firmware RF acceptance**. See [BLE status](docs/ble-status.md),
[integration](docs/ble-integration.md), and [migration](MIGRATION.md).

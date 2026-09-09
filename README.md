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

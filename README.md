# ch58x-hal

Pure Rust hardware abstraction for WCH CH58x microcontrollers.

The crate currently provides:

- compile-time selection for CH582 and CH585;
- PAC access through `ch58x_hal::pac`;
- single-owner peripheral tokens;
- a QingKe-compatible global critical-section implementation;
- system-clock setup for CH582 and CH585;
- owned digital input/output pins implementing `embedded-hal` 1.0 traits;
- bounded blocking ADC conversions, including temperature-sensor support.

## Status

The crate is under active development. APIs may change before the first stable
release.

## License

Licensed under either of
[Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at
your option.

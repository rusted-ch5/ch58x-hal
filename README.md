# ch58x-hal

Pure Rust hardware abstraction for WCH CH58x microcontrollers.

The crate currently provides:

- compile-time selection for CH582 and CH585;
- PAC access through `ch58x_hal::pac`;
- single-owner peripheral tokens;
- a QingKe-compatible global critical-section implementation;
- system-clock setup for CH582 and CH585.

## Status

The crate is under active development. APIs may change before the first stable
release.

## License

Licensed under either of
[Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at
your option.

#![no_std]
//! Experimental CH585 controller: Radio, Link Layer runner, and HCI transport.
//!
//! Applications must initialize the CH585 clocks and an Embassy time driver
//! before starting the runner. Only legacy LE 1M plaintext Peripheral operation
//! is included; encryption, pairing, Central initiation, and DLE are unsupported.

pub mod controller;
pub mod link_layer;
pub mod radio;
mod radio_dma_policy;
mod radio_phy;
mod rf_frontend;

/// Unique ownership of the Host-facing HCI queues.
pub struct ControllerToken {
    _private: (),
}
/// Unique ownership of the RF registers, DMA buffers, and BLEB interrupt.
pub struct RadioToken {
    _private: (),
}

/// Split one HAL radio token into the Host and controller ownership halves.
pub fn split(_ble: ch58x_hal::peripherals::BLE) -> (ControllerToken, RadioToken) {
    (
        ControllerToken { _private: () },
        RadioToken { _private: () },
    )
}

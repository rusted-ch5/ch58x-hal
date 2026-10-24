# BLE snapshot validation — 2026-10-24

This is an experimental source snapshot, still under code review and automated
validation. `ble-source-sha256.json` identifies the Rust source/manifests tested
in this checkout. Library checks do not produce or verify a bootable firmware.

| Evidence | Result |
| --- | --- |
| Portable core + HCI host tests | 70 passed on aarch64 macOS, Rust 1.97.1 and Rust 1.88.0 |
| Exact backend DMA/timing/W1C pure-module tests | 7 passed on the host |
| CH585 library cross-build | Passed for `riscv32imc-unknown-none-elf` |
| CH585 Clippy | Passed with `-D warnings`; Rust 1.88.0 cross-check also passed |
| CH585 rustdoc | Passed with `RUSTDOCFLAGS=-D warnings` |
| HAL chip/feature matrix | CH582 and CH585, minimal and `rt,uart`, Clippy passed |
| Firmware link/map/stack, verified flash and boot | Not run for this extracted Rust snapshot |
| Rust RF/ATT/GATT/reconnect/endurance | Not run for this extracted Rust snapshot |

Reproduce from the workspace using `cases/check-ble.sh`; set `CARGO_TARGET_DIR`
to a shared target directory to avoid duplicate caches. HAL matrix command:

```sh
cargo clippy -p ch58x-hal-rs --target riscv32imc-unknown-none-elf   --no-default-features --features ch585,rt,uart -- -D warnings
```

Repeat with `ch582`, `ch582,rt,uart`, and `ch585`. Existing HAL Rust 1.85 CI
remains separate from the BLE Rust 1.88 jobs; the local matrix above used 1.97.1.

A separate Zephyr C backend previously passed paired plaintext connection,
notification, parameter/channel-map update and reconnect tests. That evidence
motivates the capture-order correction but is **not** acceptance of this Rust
adaptation. In particular, RX-to-TX sequencing, strict outstanding-owner ACK,
reset handling, event-mask behavior under Host load, stack margin and public
Embassy integration need firmware-level verification before a release claim.

Not implemented as backend capabilities: encryption/security, Central scanning
or initiation, DLE, extended advertising, multiple connections, multi-PDU events,
or PHY updates. The peer behavior contract is in `ble-peer-contract.md`; testers
supply the Nordic implementation independently. No Nordic firmware is published.

Initial Rust 1.88 CI exposed an arm-local response lifetime and constants only
referenced by anonymous const assertions. The response now has const storage,
and the runtime wire-length policy uses the constants directly. The corrected
backend passed the local Rust 1.88 RISC-V Clippy check; earlier failures remain
in the branch CI history.

#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
export CARGO_INCREMENTAL=0
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$root/target}"
host=$(rustc -vV | sed -n 's/^host: //p')
cargo fmt --manifest-path "$root/Cargo.toml" --all -- --check
cargo test --manifest-path "$root/Cargo.toml" -p ch58x-ble-controller --target "$host"
"$root/cases/check-ble-pure.sh"
cargo clippy --manifest-path "$root/Cargo.toml" -p ch58x-ble --target riscv32imc-unknown-none-elf -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --manifest-path "$root/Cargo.toml" -p ch58x-ble --target riscv32imc-unknown-none-elf --no-deps

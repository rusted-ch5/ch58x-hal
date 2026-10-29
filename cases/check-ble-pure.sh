#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir="${CARGO_TARGET_DIR:-$root/target}/ble-pure"
mkdir -p "$build_dir"
rustc --edition 2024 --test "$root/cases/ble-pure.rs" -o "$build_dir/tests"
"$build_dir/tests"

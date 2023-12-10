#!/bin/sh
set -euv
dir="$( cd "$( dirname "$0" )" && pwd )"
cd "$dir"

#cargo build --target thumbv7em-none-eabihf --release $@
# Target is set in .cargo/config.toml
cargo build --release $@

# Target is in ELF format. Convert to binary
arm-none-eabi-objcopy -O binary ../target/thumbv7em-none-eabihf/release/sumobrain_embedded ../target/thumbv7em-none-eabihf/release/sumobrain_embedded.bin


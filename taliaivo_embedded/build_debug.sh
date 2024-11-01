#!/bin/sh
set -euv
dir="$( cd "$( dirname "$0" )" && pwd )"
cd "$dir"

# Target is set in .cargo/config.toml
cargo build $@

# Target is in ELF format. Convert to binary
arm-none-eabi-objcopy -O binary ../target/thumbv7em-none-eabihf/debug/taliaivo_embedded ../target/thumbv7em-none-eabihf/debug/taliaivo_embedded.bin


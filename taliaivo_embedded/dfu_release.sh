#!/bin/sh
set -euv
dir="$( cd "$( dirname "$0" )" && pwd )"
cd "$dir"

./dfu_firmware.sh ../target/thumbv7em-none-eabihf/release/taliaivo_embedded.bin

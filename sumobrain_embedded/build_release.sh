#!/bin/sh
set -euv
dir="$( cd "$( dirname "$0" )" && pwd )"
cd "$dir"
cargo build --target thumbv7em-none-eabihf --release $@

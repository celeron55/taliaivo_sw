#!/bin/sh
set -euv
dir="$( cd "$( dirname "$0" )" && pwd )"
cd "$dir"
cargo run --release

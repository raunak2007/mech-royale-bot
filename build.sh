#!/usr/bin/env bash
# Rebuild the static linux/amd64 binary from src/ (needs Rust; no crates are downloaded).
set -euo pipefail
cd "$(dirname "$0")"
RUSTFLAGS="-C target-feature=+crt-static" cargo build --release --bin mechbot --target x86_64-unknown-linux-gnu
strip -o mechbot target/x86_64-unknown-linux-gnu/release/mechbot
ls -la mechbot

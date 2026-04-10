#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ARGUS_BIN:-$ROOT_DIR/build/ARGUS}"

if [[ ! -x "$BIN" ]]; then
    echo "Build the binary first: cmake -S . -B build && cmake --build build -j$(nproc)" >&2
    exit 1
fi

echo "[SMOKE_SCRIPT] Running grip servo smoke test"
exec "$BIN" --motion-smoke-test --grip

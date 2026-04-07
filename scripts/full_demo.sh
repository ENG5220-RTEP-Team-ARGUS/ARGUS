#!/usr/bin/env bash
set -euo pipefail

if command -v libcamerify >/dev/null 2>&1; then
    exec libcamerify ./build/ARGUS --full-demo "$@"
fi

exec ./build/ARGUS --full-demo "$@"

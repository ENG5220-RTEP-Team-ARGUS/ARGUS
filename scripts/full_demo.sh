#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

if command -v libcamerify >/dev/null 2>&1; then
    exec libcamerify ./build/ARGUS --full-demo "$@"
fi

exec ./build/ARGUS --full-demo "$@"

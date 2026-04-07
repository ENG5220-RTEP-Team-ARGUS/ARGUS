#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

exec ./build/ARGUS --button-test "$@"

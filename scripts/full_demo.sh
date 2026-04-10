#!/usr/bin/env bash
set -euo pipefail

echo "[DEMO] full_demo.sh is deprecated; forwarding to live_test.sh"
exec ./scripts/live_test.sh "$@"

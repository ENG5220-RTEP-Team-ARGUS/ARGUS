#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

requested_backend="${ARGUS_CAMERA_BACKEND:-}"
args=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --camera-backend)
            if [[ $# -lt 2 ]]; then
                echo "missing value for --camera-backend" >&2
                exit 1
            fi
            requested_backend="$2"
            args+=("$1" "$2")
            shift 2
            ;;
        *)
            args+=("$1")
            shift
            ;;
    esac
done

if [[ -n "$requested_backend" ]]; then
    if [[ "$requested_backend" == "opencv" || "$requested_backend" == "videocapture" ]]; then
        if command -v libcamerify >/dev/null 2>&1; then
            exec libcamerify ./build/ARGUS --live-test "${args[@]}"
        fi
    fi

    exec ./build/ARGUS --live-test "${args[@]}"
fi

echo "[LIVE_TEST] trying compliance camera backend first"
if ./build/ARGUS --live-test --camera-backend libcamera2opencv "${args[@]}"; then
    exit 0
fi

echo "[LIVE_TEST] compliance camera backend failed; falling back to opencv"
if command -v libcamerify >/dev/null 2>&1; then
    exec libcamerify ./build/ARGUS --live-test --camera-backend opencv "${args[@]}"
fi

exec ./build/ARGUS --live-test --camera-backend opencv "${args[@]}"

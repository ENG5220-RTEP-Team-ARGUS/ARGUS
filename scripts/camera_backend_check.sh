#!/usr/bin/env bash
set -euo pipefail

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
            exec libcamerify ./build/ARGUS --camera-backend-check "${args[@]}"
        fi
    fi

    exec ./build/ARGUS --camera-backend-check "${args[@]}"
fi

echo "[CAMERA_CHECK] trying compliance camera backend first"
if ./build/ARGUS --camera-backend-check --camera-backend libcamera2opencv "${args[@]}"; then
    echo "[CAMERA_CHECK] compliance backend passed"
else
    echo "[CAMERA_CHECK] compliance backend failed"
fi

echo "[CAMERA_CHECK] validating opencv fallback backend"
if command -v libcamerify >/dev/null 2>&1; then
    exec libcamerify ./build/ARGUS --camera-backend-check --camera-backend opencv "${args[@]}"
fi

exec ./build/ARGUS --camera-backend-check --camera-backend opencv "${args[@]}"

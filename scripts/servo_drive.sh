#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ARGUS_BIN:-$ROOT_DIR/build/ARGUS}"
STEP="${ARGUS_SERVO_STEP:-5}"
EXIT_HOME_WAIT_S="${ARGUS_SERVO_EXIT_HOME_WAIT_S:-1.0}"

if [[ ! -x "$BIN" ]]; then
    echo "Build the binary first: cmake -S . -B build && cmake --build build -j$(nproc)" >&2
    exit 1
fi

if ! [[ "$STEP" =~ ^[0-9]+$ ]] || (( STEP <= 0 )); then
    echo "ARGUS_SERVO_STEP must be a positive integer (current: $STEP)" >&2
    exit 1
fi

if ! [[ "$EXIT_HOME_WAIT_S" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "ARGUS_SERVO_EXIT_HOME_WAIT_S must be a non-negative number (current: $EXIT_HOME_WAIT_S)" >&2
    exit 1
fi

if ! [[ -t 0 ]] || ! [[ -t 1 ]]; then
    echo "servo_drive.sh requires an interactive terminal (TTY)." >&2
    exit 1
fi

clamp_deg() {
    local value="$1"
    if (( value < -90 )); then
        echo "-90"
    elif (( value > 90 )); then
        echo "90"
    else
        echo "$value"
    fi
}

BASE=0
LOWER=0
UPPER=0
GRIP=0

KEY_BASE_LEFT=""
KEY_BASE_RIGHT=""
KEY_UPPER_FORWARD=""
KEY_UPPER_BACKWARD=""
KEY_LOWER_UP=""
KEY_LOWER_DOWN=""
KEY_GRIP_OPEN=""
KEY_GRIP_CLOSE=""

send_cmd() {
    local cmd="$1"
    printf '%s\n' "$cmd" >&"${SERVO_CONSOLE[1]}"
}

print_status() {
    echo "[DRIVE] base=$BASE lower=$LOWER upper=$UPPER grip=$GRIP step=$STEP"
}

set_qwerty_keymap() {
    KEY_BASE_LEFT="d"
    KEY_BASE_RIGHT="a"
    KEY_UPPER_FORWARD="w"
    KEY_UPPER_BACKWARD="s"
    KEY_LOWER_UP="i"
    KEY_LOWER_DOWN="k"
    KEY_GRIP_OPEN="l"
    KEY_GRIP_CLOSE="j"
}

set_azerty_keymap() {
    KEY_BASE_LEFT="d"
    KEY_BASE_RIGHT="q"
    KEY_UPPER_FORWARD="z"
    KEY_UPPER_BACKWARD="s"
    KEY_LOWER_UP="i"
    KEY_LOWER_DOWN="k"
    KEY_GRIP_OPEN="l"
    KEY_GRIP_CLOSE="j"
}

prompt_single_key() {
    local label="$1"
    local default_key="$2"
    local value=""
    while true; do
        read -rp "[DRIVE] $label key [$default_key]: " value
        value="${value:-$default_key}"
        value="${value,,}"
        if [[ ${#value} -eq 1 ]]; then
            printf '%s' "$value"
            return 0
        fi
        echo "[DRIVE] Please enter a single character."
    done
}

validate_keymap() {
    local keys=(
        "$KEY_BASE_LEFT"
        "$KEY_BASE_RIGHT"
        "$KEY_UPPER_FORWARD"
        "$KEY_UPPER_BACKWARD"
        "$KEY_LOWER_UP"
        "$KEY_LOWER_DOWN"
        "$KEY_GRIP_OPEN"
        "$KEY_GRIP_CLOSE"
    )
    local reserved=("h" "r" "x" "+" "-" "=" "_")
    declare -A seen=()
    local key=""
    local r=""

    for key in "${keys[@]}"; do
        if [[ ${#key} -ne 1 ]]; then
            echo "[DRIVE] Invalid key binding '$key' (must be one character)." >&2
            return 1
        fi
        if [[ "$key" =~ [[:space:]] ]]; then
            echo "[DRIVE] Key bindings cannot be whitespace." >&2
            return 1
        fi
        if [[ -n "${seen[$key]:-}" ]]; then
            echo "[DRIVE] Duplicate key binding '$key'." >&2
            return 1
        fi
        seen[$key]=1
    done

    for key in "${keys[@]}"; do
        for r in "${reserved[@]}"; do
            if [[ "$key" == "$r" ]]; then
                echo "[DRIVE] Key '$key' is reserved (home/status/step/exit)." >&2
                return 1
            fi
        done
    done

    return 0
}

print_keymap() {
    echo "[DRIVE] Keymap:"
    echo "[DRIVE]   $KEY_BASE_LEFT/$KEY_BASE_RIGHT = base left/right"
    echo "[DRIVE]   $KEY_UPPER_FORWARD/$KEY_UPPER_BACKWARD = upper forward/back"
    echo "[DRIVE]   $KEY_LOWER_UP/$KEY_LOWER_DOWN = lower up/down"
    echo "[DRIVE]   $KEY_GRIP_OPEN/$KEY_GRIP_CLOSE = grip open/close"
    echo "[DRIVE]   h=home r=status +/-=step x=exit"
}

configure_keymap() {
    local mode="${ARGUS_SERVO_KEYMAP:-}"
    local choice=""

    if [[ -z "$mode" ]]; then
        echo "[DRIVE] Select keymap preset:"
        echo "[DRIVE]   1) azerty (d/q z/s i/k l/j)"
        echo "[DRIVE]   2) qwerty (d/a w/s i/k l/j)"
        echo "[DRIVE]   3) custom"
        read -rp "[DRIVE] choice [1/2/3] (default: 1): " choice
        choice="${choice:-1}"
        case "$choice" in
            1) mode="azerty" ;;
            2) mode="qwerty" ;;
            3) mode="custom" ;;
            *)
                echo "[DRIVE] Unknown choice '$choice', defaulting to azerty."
                mode="azerty"
                ;;
        esac
    fi

    mode="${mode,,}"
    case "$mode" in
        azerty)
            set_azerty_keymap
            ;;
        qwerty)
            set_qwerty_keymap
            ;;
        custom)
            set_azerty_keymap
            echo "[DRIVE] Custom keymap setup:"
            KEY_BASE_LEFT="$(prompt_single_key "base left" "$KEY_BASE_LEFT")"
            KEY_BASE_RIGHT="$(prompt_single_key "base right" "$KEY_BASE_RIGHT")"
            KEY_UPPER_FORWARD="$(prompt_single_key "upper forward" "$KEY_UPPER_FORWARD")"
            KEY_UPPER_BACKWARD="$(prompt_single_key "upper backward" "$KEY_UPPER_BACKWARD")"
            KEY_LOWER_UP="$(prompt_single_key "lower up" "$KEY_LOWER_UP")"
            KEY_LOWER_DOWN="$(prompt_single_key "lower down" "$KEY_LOWER_DOWN")"
            KEY_GRIP_OPEN="$(prompt_single_key "grip open" "$KEY_GRIP_OPEN")"
            KEY_GRIP_CLOSE="$(prompt_single_key "grip close" "$KEY_GRIP_CLOSE")"
            ;;
        *)
            echo "[DRIVE] Unknown ARGUS_SERVO_KEYMAP='$mode'. Use azerty, qwerty, or custom." >&2
            exit 1
            ;;
    esac

    if ! validate_keymap; then
        echo "[DRIVE] Invalid keymap configuration." >&2
        exit 1
    fi
}

echo "[DRIVE] Controller-style servo teleop"
echo "[DRIVE] Logical range is clamped to -90..+90"
echo

configure_keymap
print_keymap
echo

coproc SERVO_CONSOLE { "$BIN" --servo-console; }

if [[ -z "${SERVO_CONSOLE_PID:-}" ||
      -z "${SERVO_CONSOLE[0]-}" ||
      -z "${SERVO_CONSOLE[1]-}" ]]; then
    echo "[DRIVE] Failed to start servo console subprocess." >&2
    exit 1
fi

if ! : <&"${SERVO_CONSOLE[0]}" 2>/dev/null; then
    echo "[DRIVE] Servo console output pipe is unavailable (bad file descriptor)." >&2
    exit 1
fi

if ! : >&"${SERVO_CONSOLE[1]}" 2>/dev/null; then
    echo "[DRIVE] Servo console input pipe is unavailable (bad file descriptor)." >&2
    exit 1
fi

forward_servo_console_output() {
    local line=""
    while IFS= read -r -u "${SERVO_CONSOLE[0]}" line 2>/dev/null; do
        printf '%s\n' "$line"
    done
}

forward_servo_console_output &
SERVO_LOG_PID=$!

cleanup() {
    local old_stty="$1"
    stty "$old_stty" 2>/dev/null || true
    if [[ -n "${SERVO_CONSOLE_PID:-}" ]]; then
        printf 'exit\n' >&"${SERVO_CONSOLE[1]}" 2>/dev/null || true
    fi
    if [[ -n "${SERVO_LOG_PID:-}" ]]; then
        kill "$SERVO_LOG_PID" 2>/dev/null || true
        wait "$SERVO_LOG_PID" 2>/dev/null || true
    fi
    if [[ -n "${SERVO_CONSOLE_PID:-}" ]]; then
        wait "$SERVO_CONSOLE_PID" 2>/dev/null || true
    fi
}

OLD_STTY="$(stty -g)"
trap 'cleanup "$OLD_STTY"' EXIT
trap 'exit 130' INT TERM
stty -echo -icanon

sleep 0.2
print_status

while true; do
    IFS= read -rsn1 key || break

    key="${key,,}"

    case "$key" in
        h)
            BASE=0
            LOWER=0
            UPPER=0
            GRIP=0
            send_cmd "home"
            print_status
            ;;
        r)
            send_cmd "status"
            print_status
            ;;
        +|=)
            (( STEP += 1 ))
            echo "[DRIVE] step=$STEP"
            ;;
        -|_)
            if (( STEP > 1 )); then
                (( STEP -= 1 ))
            fi
            echo "[DRIVE] step=$STEP"
            ;;
        x)
            BASE=0
            LOWER=0
            UPPER=0
            GRIP=0
            send_cmd "home"
            print_status
            if [[ "$EXIT_HOME_WAIT_S" != "0" ]]; then
                echo "[DRIVE] waiting ${EXIT_HOME_WAIT_S}s for home settle"
                sleep "$EXIT_HOME_WAIT_S"
            fi
            echo "[DRIVE] exit"
            break
            ;;
        *)
            if [[ "$key" == "$KEY_BASE_LEFT" ]]; then
                BASE="$(clamp_deg "$((BASE - STEP))")"
                send_cmd "base $BASE"
                print_status
            elif [[ "$key" == "$KEY_BASE_RIGHT" ]]; then
                BASE="$(clamp_deg "$((BASE + STEP))")"
                send_cmd "base $BASE"
                print_status
            elif [[ "$key" == "$KEY_UPPER_FORWARD" ]]; then
                UPPER="$(clamp_deg "$((UPPER + STEP))")"
                send_cmd "upper $UPPER"
                print_status
            elif [[ "$key" == "$KEY_UPPER_BACKWARD" ]]; then
                UPPER="$(clamp_deg "$((UPPER - STEP))")"
                send_cmd "upper $UPPER"
                print_status
            elif [[ "$key" == "$KEY_LOWER_UP" ]]; then
                LOWER="$(clamp_deg "$((LOWER + STEP))")"
                send_cmd "lower $LOWER"
                print_status
            elif [[ "$key" == "$KEY_LOWER_DOWN" ]]; then
                LOWER="$(clamp_deg "$((LOWER - STEP))")"
                send_cmd "lower $LOWER"
                print_status
            elif [[ "$key" == "$KEY_GRIP_OPEN" ]]; then
                GRIP="$(clamp_deg "$((GRIP + STEP))")"
                send_cmd "grip $GRIP"
                print_status
            elif [[ "$key" == "$KEY_GRIP_CLOSE" ]]; then
                GRIP="$(clamp_deg "$((GRIP - STEP))")"
                send_cmd "grip $GRIP"
                print_status
            fi
            ;;
    esac
done

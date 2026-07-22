#!/usr/bin/env bash
# Install PlatformIO (if needed) and build the ESP32-C3 firmware.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

ENV_NAME="${PIO_ENV:-esp32c3-supermini}"
UPLOAD=0
MONITOR=0

usage() {
  cat <<'EOF'
Usage: ./setup_and_build.sh [options]

Options:
  --upload [PORT]   Flash after a successful build (default port: /dev/ttyACM0)
  --monitor         Open serial monitor after build/upload
  --env NAME        PlatformIO env (default: esp32c3-supermini)
  -h, --help        Show this help

Examples:
  ./setup_and_build.sh
  ./setup_and_build.sh --upload
  ./setup_and_build.sh --upload /dev/ttyUSB0 --monitor
EOF
}

UPLOAD_PORT="/dev/ttyACM0"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --upload)
      UPLOAD=1
      if [[ "${2:-}" != "" && "${2:-}" != --* ]]; then
        UPLOAD_PORT="$2"
        shift
      fi
      ;;
    --monitor)
      MONITOR=1
      ;;
    --env)
      ENV_NAME="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

log() { printf '==> %s\n' "$*"; }

ensure_python() {
  if command -v python3 >/dev/null 2>&1; then
    return
  fi
  echo "python3 is required but not found." >&2
  exit 1
}

ensure_pio() {
  if command -v pio >/dev/null 2>&1; then
    PIO=(pio)
    return
  fi
  if [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    PIO=("$HOME/.platformio/penv/bin/pio")
    return
  fi
  if [[ -x "$HOME/.local/bin/pio" ]]; then
    PIO=("$HOME/.local/bin/pio")
    return
  fi

  log "PlatformIO not found — installing with pip (user install)..."
  ensure_python
  python3 -m pip install --user -U platformio
  export PATH="$HOME/.local/bin:$PATH"
  if ! command -v pio >/dev/null 2>&1; then
    echo "pio still not on PATH after install. Add \$HOME/.local/bin to PATH." >&2
    exit 1
  fi
  PIO=(pio)
}

ensure_secrets() {
  if [[ ! -f include/secrets.h ]]; then
    if [[ -f include/secrets.h.example ]]; then
      log "Creating include/secrets.h from example — edit WiFi/MQTT credentials before flashing."
      cp include/secrets.h.example include/secrets.h
    else
      echo "Missing include/secrets.h (and no secrets.h.example)." >&2
      exit 1
    fi
  fi
}

ensure_python
ensure_pio
ensure_secrets

log "PlatformIO: $("${PIO[@]}" --version)"
log "Building env '${ENV_NAME}'..."
"${PIO[@]}" run -e "$ENV_NAME"

FIRMWARE=".pio/build/${ENV_NAME}/firmware.bin"
if [[ -f "$FIRMWARE" ]]; then
  log "Build OK: $FIRMWARE"
else
  echo "Build finished but firmware binary not found at $FIRMWARE" >&2
  exit 1
fi

if [[ "$UPLOAD" -eq 1 ]]; then
  log "Uploading to ${UPLOAD_PORT}..."
  "${PIO[@]}" run -e "$ENV_NAME" -t upload --upload-port "$UPLOAD_PORT"
fi

if [[ "$MONITOR" -eq 1 ]]; then
  log "Opening serial monitor (Ctrl+] to exit)..."
  if [[ "$UPLOAD" -eq 1 ]]; then
    "${PIO[@]}" device monitor -e "$ENV_NAME" --port "$UPLOAD_PORT"
  else
    "${PIO[@]}" device monitor -e "$ENV_NAME"
  fi
fi

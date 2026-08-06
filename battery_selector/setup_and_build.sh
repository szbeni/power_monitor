#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

UPLOAD=0
MONITOR=0
OTA=0
UPLOAD_PORT="/dev/ttyUSB0"
OTA_IP=""
ENV_NAME="esp32dev"

usage() {
  cat <<'EOF'
Usage: ./setup_and_build.sh [--upload [PORT]] [--ota IP] [--monitor]

  --upload [PORT]   USB flash (default /dev/ttyUSB0)
  --ota IP          ArduinoOTA upload (espota) to device IP
  --monitor         Serial monitor after upload

Examples:
  ./setup_and_build.sh --upload /dev/ttyUSB0 --monitor
  ./setup_and_build.sh --ota 192.168.1.50
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --upload)
      UPLOAD=1
      if [[ "${2:-}" != "" && "${2:-}" != --* ]]; then
        UPLOAD_PORT="$2"
        shift
      fi
      ;;
    --ota)
      OTA=1
      if [[ "${2:-}" == "" || "${2:-}" == --* ]]; then
        echo "error: --ota requires an IP address" >&2
        exit 1
      fi
      OTA_IP="$2"
      shift
      ;;
    --monitor) MONITOR=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

if ! command -v pio >/dev/null 2>&1; then
  if [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    PATH="$HOME/.platformio/penv/bin:$PATH"
  elif [[ -x "$HOME/.local/bin/pio" ]]; then
    PATH="$HOME/.local/bin:$PATH"
  else
    echo "==> Installing PlatformIO..."
    python3 -m pip install -U platformio
    export PATH="$HOME/.local/bin:$PATH"
  fi
fi

if [[ ! -f include/secrets.h ]]; then
  echo "==> Creating include/secrets.h from example"
  cp include/secrets.h.example include/secrets.h
fi

echo "==> Building battery_selector ($ENV_NAME)"
pio run -e "$ENV_NAME"

if [[ "$UPLOAD" -eq 1 ]]; then
  echo "==> Uploading (USB) to $UPLOAD_PORT"
  pio run -e "$ENV_NAME" -t upload --upload-port "$UPLOAD_PORT"
fi

if [[ "$OTA" -eq 1 ]]; then
  echo "==> OTA upload to $OTA_IP (env esp32-ota)"
  echo "    Auth must match OTA_PASSWORD in include/secrets.h"
  echo "    (default auth in platformio.ini: ota_change_me)"
  pio run -e esp32-ota -t upload --upload-port "$OTA_IP"
fi

if [[ "$MONITOR" -eq 1 ]]; then
  pio device monitor --port "$UPLOAD_PORT" -b 115200
fi

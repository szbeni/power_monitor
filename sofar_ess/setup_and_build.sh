#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

UPLOAD=0
MONITOR=0
UPLOAD_PORT="/dev/ttyUSB0"

usage() {
  cat <<'EOF'
Usage: ./setup_and_build.sh [--upload [PORT]] [--monitor]

Default upload port for NodeMCU is /dev/ttyUSB0.
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
    --monitor) MONITOR=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown: $1" >&2; usage >&2; exit 1 ;;
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
    python3 -m pip install --user -U platformio
    export PATH="$HOME/.local/bin:$PATH"
  fi
fi

if [[ ! -f include/secrets.h ]]; then
  echo "==> Creating include/secrets.h from example"
  cp include/secrets.h.example include/secrets.h
fi

echo "==> Building sofar_ess (nodemcuv2)"
pio run

if [[ "$UPLOAD" -eq 1 ]]; then
  echo "==> Uploading to $UPLOAD_PORT"
  pio run -t upload --upload-port "$UPLOAD_PORT"
fi

if [[ "$MONITOR" -eq 1 ]]; then
  pio device monitor --port "$UPLOAD_PORT" -b 115200
fi

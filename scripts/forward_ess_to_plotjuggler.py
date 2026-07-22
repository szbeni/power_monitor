#!/usr/bin/env python3
"""Forward ESS load2/power MQTT to PlotJuggler over UDP (localhost).

Uses mosquitto_sub (no Python MQTT package required). Credentials default
from include/secrets.h / include/config.h when present.

PlotJuggler:
  Streaming → UDP Server (JSON) → listen on --pj-port (default 9870)

Examples:
  ./scripts/forward_ess_to_plotjuggler.py
  ./scripts/forward_ess_to_plotjuggler.py --pj-port 9870
  ./scripts/forward_ess_to_plotjuggler.py --host 10.1.1.1 --user caravan --password secret
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SECRETS_H = ROOT / "include" / "secrets.h"
CONFIG_H = ROOT / "include" / "config.h"


def _define_str(text: str, name: str, default: str = "") -> str:
    m = re.search(rf'#define\s+{name}\s+"([^"]*)"', text)
    return m.group(1) if m else default


def _define_int(text: str, name: str, default: int) -> int:
    m = re.search(rf"#define\s+{name}\s+(\d+)", text)
    return int(m.group(1)) if m else default


def load_defaults() -> dict:
    out = {
        "host": "127.0.0.1",
        "port": 1883,
        "user": "",
        "password": "",
        "topic": "power_monitor/jsy/load2/power",
    }
    if CONFIG_H.is_file():
        cfg = CONFIG_H.read_text(encoding="utf-8", errors="replace")
        root = _define_str(cfg, "MQTT_TOPIC_ROOT", "power_monitor/jsy")
        out["topic"] = f"{root}/load2/power"
    if SECRETS_H.is_file():
        sec = SECRETS_H.read_text(encoding="utf-8", errors="replace")
        out["host"] = _define_str(sec, "MQTT_HOST", out["host"]) or out["host"]
        out["port"] = _define_int(sec, "MQTT_PORT", out["port"])
        out["user"] = _define_str(sec, "MQTT_USER", "")
        out["password"] = _define_str(sec, "MQTT_PASSWORD", "")
    return out


def parse_power(line: str) -> float | None:
    text = line.strip()
    if not text:
        return None
    # ESS is a plain float; tolerate "0.000 123" style payloads if present.
    token = text.split()[0]
    try:
        return float(token)
    except ValueError:
        return None


def main() -> int:
    defaults = load_defaults()
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--host", default=defaults["host"], help="MQTT broker host")
    ap.add_argument("--port", type=int, default=defaults["port"], help="MQTT broker port")
    ap.add_argument("--user", default=defaults["user"])
    ap.add_argument("--password", default=defaults["password"])
    ap.add_argument("--topic", default=defaults["topic"])
    ap.add_argument("--pj-host", default="127.0.0.1", help="PlotJuggler UDP host")
    ap.add_argument("--pj-port", type=int, default=9870, help="PlotJuggler UDP port")
    ap.add_argument(
        "--series",
        default="ess_power_w",
        help="JSON key / series name sent to PlotJuggler",
    )
    ap.add_argument(
        "--include-t",
        action="store_true",
        help="Also send relative time field 't' (seconds since start)",
    )
    args = ap.parse_args()

    mosq = shutil.which("mosquitto_sub")
    if not mosq:
        print("mosquitto_sub not found on PATH (install mosquitto clients)", file=sys.stderr)
        return 2

    cmd = [
        mosq,
        "-h",
        args.host,
        "-p",
        str(args.port),
        "-t",
        args.topic,
    ]
    if args.user:
        cmd.extend(["-u", args.user, "-P", args.password])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.pj_host, args.pj_port)
    t0 = time.monotonic()
    count = 0

    print(f"MQTT {args.topic} @ {args.host}:{args.port}")
    print(f"→ UDP JSON {args.pj_host}:{args.pj_port}  series={args.series}")
    print("Start PlotJuggler UDP Server (JSON) on that port, then Ctrl+C to stop.")

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except OSError as exc:
        print(f"failed to start mosquitto_sub: {exc}", file=sys.stderr)
        return 2

    assert proc.stdout is not None
    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                err = proc.stderr.read() if proc.stderr else ""
                if err.strip():
                    print(err.strip(), file=sys.stderr)
                print("mosquitto_sub exited", file=sys.stderr)
                return 1

            power = parse_power(line)
            if power is None:
                continue

            pkt: dict = {args.series: power}
            if args.include_t:
                pkt["t"] = time.monotonic() - t0

            sock.sendto(json.dumps(pkt, separators=(",", ":")).encode("utf-8"), dest)
            count += 1
            if count == 1 or count % 40 == 0:
                print(f"forwarded #{count}  {args.series}={power:.3f}")
    except KeyboardInterrupt:
        print(f"\nstopped after {count} packets.")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        sock.close()


if __name__ == "__main__":
    sys.exit(main())

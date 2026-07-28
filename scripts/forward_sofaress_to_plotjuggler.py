#!/usr/bin/env python3
"""Forward sofar_ess MQTT → PlotJuggler UDP JSON in realtime.

Prefers paho-mqtt (callback → UDP immediately). Falls back to
mosquitto_sub wrapped in stdbuf -oL (piped mosquitto_sub is otherwise
block-buffered and looks multi-second laggy).

Defaults from sofar_ess/include/secrets.h + DEVICE_NAME in config.h.

PlotJuggler:
  Streaming → UDP Server → JSON → port --pj-port (default 9870)

Note: the ESP only publishes live ess/* about every ESS_LOOP_INTERVAL_MS
(500 ms). This script cannot invent faster samples than MQTT provides.

Examples:
  ./scripts/forward_sofaress_to_plotjuggler.py
  ./scripts/forward_sofaress_to_plotjuggler.py --live-only
  pip install paho-mqtt   # recommended for lowest latency
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
SOFAR_SECRETS = ROOT / "sofar_ess" / "include" / "secrets.h"
SOFAR_CONFIG = ROOT / "sofar_ess" / "include" / "config.h"
ROOT_SECRETS = ROOT / "include" / "secrets.h"


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
        "device": "sofaress",
    }
    if SOFAR_CONFIG.is_file():
        cfg = SOFAR_CONFIG.read_text(encoding="utf-8", errors="replace")
        out["device"] = _define_str(cfg, "DEVICE_NAME", out["device"]) or out["device"]

    for path in (SOFAR_SECRETS, ROOT_SECRETS):
        if not path.is_file():
            continue
        sec = path.read_text(encoding="utf-8", errors="replace")
        out["host"] = _define_str(sec, "MQTT_HOST", out["host"]) or out["host"]
        out["port"] = _define_int(sec, "MQTT_PORT", out["port"])
        out["user"] = _define_str(sec, "MQTT_USER", out["user"])
        out["password"] = _define_str(sec, "MQTT_PASSWORD", out["password"])
        break
    return out


def topic_to_series(device: str, topic: str) -> str:
    if topic.startswith(device + "/"):
        return topic
    return f"{device}/{topic}"


def flatten_payload(device: str, topic: str, payload: str) -> dict[str, float]:
    text = payload.strip()
    if not text:
        return {}

    if topic.endswith("/state") or text.startswith("{"):
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            obj = None
        if isinstance(obj, dict):
            out: dict[str, float] = {}
            modes = {
                "standby": 0.0,
                "charge": 1.0,
                "discharge": 2.0,
                "auto": 3.0,
                "unknown": -1.0,
            }
            for key, val in obj.items():
                name = f"{device}/state/{key}"
                if isinstance(val, bool):
                    out[name] = 1.0 if val else 0.0
                elif isinstance(val, (int, float)):
                    out[name] = float(val)
                elif isinstance(val, str):
                    if key == "sofar_mode" and val in modes:
                        out[name] = modes[val]
                    elif val in ("true", "false", "online", "offline"):
                        out[name] = 1.0 if val in ("true", "online") else 0.0
            return out

    low = text.lower()
    series = topic_to_series(device, topic)
    if low in ("true", "online", "on", "1"):
        return {series: 1.0}
    if low in ("false", "offline", "off", "0"):
        return {series: 0.0}
    modes = {"standby": 0.0, "charge": 1.0, "discharge": 2.0, "auto": 3.0, "unknown": -1.0}
    if low in modes:
        return {series: modes[low]}

    token = text.split()[0]
    try:
        return {series: float(token)}
    except ValueError:
        return {}


class UdpForwarder:
    def __init__(self, host: str, port: int, include_t: bool) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Smaller buffer / no coalescing — send ASAP
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
        except OSError:
            pass
        self.dest = (host, port)
        self.include_t = include_t
        self.t0 = time.monotonic()
        self.count = 0

    def send(self, series: dict[str, float]) -> None:
        if not series:
            return
        pkt: dict = dict(series)
        # PlotJuggler uses receive time; also stamp for aligned multi-series.
        now = time.time()
        pkt["timestamp"] = now
        if self.include_t:
            pkt["t"] = time.monotonic() - self.t0
        self.sock.sendto(json.dumps(pkt, separators=(",", ":")).encode("utf-8"), self.dest)
        self.count += 1
        if self.count == 1 or self.count % 100 == 0:
            preview = ", ".join(f"{k.split('/')[-1]}={v:g}" for k, v in list(series.items())[:3])
            print(f"#{self.count}  {preview}", flush=True)

    def close(self) -> None:
        self.sock.close()


def run_paho(args: argparse.Namespace, topic: str, fwd: UdpForwarder) -> int:
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        return -1

    def on_message(_client: object, _userdata: object, msg: object) -> None:
        payload = msg.payload.decode("utf-8", errors="replace")  # type: ignore[attr-defined]
        series = flatten_payload(args.device, msg.topic, payload)  # type: ignore[attr-defined]
        fwd.send(series)

    def on_connect(client: object, _userdata: object, _flags: object, reason_code: object, *_args: object) -> None:
        rc = int(getattr(reason_code, "value", reason_code))
        if rc != 0:
            print(f"MQTT connect failed rc={rc}", file=sys.stderr, flush=True)
            return
        print(f"MQTT connected (paho)  sub {topic}", flush=True)
        client.subscribe(topic, qos=0)  # type: ignore[attr-defined]

    # paho 1.x vs 2.x
    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,  # type: ignore[attr-defined]
            client_id=f"pj-fwd-{int(time.time()) % 100000}",
            protocol=mqtt.MQTTv311,
        )
        client.on_connect = on_connect
    except (AttributeError, TypeError):
        client = mqtt.Client(client_id=f"pj-fwd-{int(time.time()) % 100000}")
        def on_connect_v1(client: object, _u: object, _f: object, rc: int) -> None:
            if rc != 0:
                print(f"MQTT connect failed rc={rc}", file=sys.stderr, flush=True)
                return
            print(f"MQTT connected (paho)  sub {topic}", flush=True)
            client.subscribe(topic, qos=0)  # type: ignore[attr-defined]
        client.on_connect = on_connect_v1  # type: ignore[assignment]

    client.on_message = on_message
    if args.user:
        client.username_pw_set(args.user, args.password)

    client.connect(args.host, args.port, keepalive=30)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print(f"\nstopped after {fwd.count} packets.", flush=True)
        return 0
    finally:
        try:
            client.disconnect()
        except Exception:
            pass
    return 0


def run_mosquitto(args: argparse.Namespace, topic: str, fwd: UdpForwarder) -> int:
    mosq = shutil.which("mosquitto_sub")
    if not mosq:
        print("Need paho-mqtt or mosquitto_sub on PATH", file=sys.stderr)
        print("  pip install paho-mqtt", file=sys.stderr)
        return 2

    # Line-buffer mosquitto_sub — without this, piped stdout is block-buffered (~seconds lag).
    stdbuf = shutil.which("stdbuf")
    cmd: list[str] = []
    if stdbuf:
        cmd.extend([stdbuf, "-oL", "-eL"])
    else:
        print("warning: stdbuf not found — mosquitto_sub may lag; install paho-mqtt", file=sys.stderr)

    cmd.extend(
        [
            mosq,
            "-h",
            args.host,
            "-p",
            str(args.port),
            "-t",
            topic,
            "-q",
            "0",
            "-v",
        ]
    )
    if args.user:
        cmd.extend(["-u", args.user, "-P", args.password])

    print(f"MQTT via mosquitto_sub (+stdbuf)  sub {topic}", flush=True)
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
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
            line = line.rstrip("\n")
            parts = line.split(None, 1)
            if len(parts) < 2:
                continue
            series = flatten_payload(args.device, parts[0], parts[1])
            fwd.send(series)
    except KeyboardInterrupt:
        print(f"\nstopped after {fwd.count} packets.", flush=True)
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()


def main() -> int:
    defaults = load_defaults()
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--host", default=defaults["host"])
    ap.add_argument("--port", type=int, default=defaults["port"])
    ap.add_argument("--user", default=defaults["user"])
    ap.add_argument("--password", default=defaults["password"])
    ap.add_argument("--device", default=defaults["device"])
    ap.add_argument(
        "--live-only",
        action="store_true",
        help="Subscribe only to <device>/ess/# (skip slow state/HA noise)",
    )
    ap.add_argument("--topic", default="", help="Override subscribe topic")
    ap.add_argument("--pj-host", default="127.0.0.1")
    ap.add_argument("--pj-port", type=int, default=9870)
    ap.add_argument(
        "--include-t",
        action="store_true",
        default=True,
        help="Include relative t= seconds (default on)",
    )
    ap.add_argument("--no-t", action="store_false", dest="include_t")
    ap.add_argument(
        "--mosquitto",
        action="store_true",
        help="Force mosquitto_sub instead of paho-mqtt",
    )
    args = ap.parse_args()

    if args.topic:
        topic = args.topic
    elif args.live_only:
        topic = f"{args.device}/ess/#"
    else:
        topic = f"{args.device}/#"

    fwd = UdpForwarder(args.pj_host, args.pj_port, args.include_t)
    print(f"→ UDP JSON {args.pj_host}:{args.pj_port}  (device MQTT ~500 ms)", flush=True)
    print("PlotJuggler: Streaming → UDP Server → JSON. Ctrl+C to stop.", flush=True)

    try:
        if not args.mosquitto:
            rc = run_paho(args, topic, fwd)
            if rc >= 0:
                return rc
            print("paho-mqtt not installed — falling back to mosquitto_sub", flush=True)
            print("  for best realtime: pip install paho-mqtt", flush=True)
        return run_mosquitto(args, topic, fwd)
    finally:
        fwd.close()


if __name__ == "__main__":
    sys.exit(main())

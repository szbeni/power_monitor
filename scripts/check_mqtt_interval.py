#!/usr/bin/env python3
"""Measure MQTT publish intervals for the ESS load2 power topic.

Uses mosquitto_sub (no Python MQTT package required). Credentials default
from include/secrets.h / include/config.h when present.

Examples:
  ./scripts/check_mqtt_interval.py
  ./scripts/check_mqtt_interval.py --host 10.1.1.1 --user caravan --password secret
  ./scripts/check_mqtt_interval.py --seconds 20 --expect-ms 250
"""

from __future__ import annotations

import argparse
import re
import shutil
import statistics
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
        "expect_ms": 250.0,
    }
    if CONFIG_H.is_file():
        cfg = CONFIG_H.read_text(encoding="utf-8", errors="replace")
        root = _define_str(cfg, "MQTT_TOPIC_ROOT", "power_monitor/jsy")
        out["topic"] = f"{root}/load2/power"
        out["expect_ms"] = float(_define_int(cfg, "ESS_PUBLISH_INTERVAL_MS", 250))
    if SECRETS_H.is_file():
        sec = SECRETS_H.read_text(encoding="utf-8", errors="replace")
        out["host"] = _define_str(sec, "MQTT_HOST", out["host"]) or out["host"]
        out["port"] = _define_int(sec, "MQTT_PORT", out["port"])
        out["user"] = _define_str(sec, "MQTT_USER", "")
        out["password"] = _define_str(sec, "MQTT_PASSWORD", "")
    return out


def percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p
    f = int(k)
    c = min(f + 1, len(sorted_vals) - 1)
    if f == c:
        return sorted_vals[f]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


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
    ap.add_argument("--topic", default=defaults["topic"])
    ap.add_argument("--expect-ms", type=float, default=defaults["expect_ms"], help="Expected interval (ms)")
    ap.add_argument("--seconds", type=float, default=15.0, help="How long to sample")
    ap.add_argument("--count", type=int, default=0, help="Stop after N messages (0 = use --seconds)")
    ap.add_argument("--tolerance-ms", type=float, default=50.0, help="PASS if |mean-expect| <= this")
    ap.add_argument("--show", type=int, default=12, help="Print first N intervals")
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

    print(f"topic={args.topic}")
    print(f"broker={args.host}:{args.port}  expect={args.expect_ms:.1f} ms")
    print(
        f"sampling for {args.seconds:.1f}s"
        + (f" or {args.count} msgs" if args.count else "")
        + " ..."
    )

    times: list[float] = []
    payloads: list[str] = []
    deadline = time.monotonic() + args.seconds

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
        while time.monotonic() < deadline:
            if args.count and len(times) >= args.count:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            line = proc.stdout.readline()
            if not line:
                # process exited
                err = proc.stderr.read() if proc.stderr else ""
                if err.strip():
                    print(err.strip(), file=sys.stderr)
                break
            times.append(time.monotonic())
            payloads.append(line.rstrip("\n"))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    if len(times) < 2:
        print(f"messages={len(times)} — need at least 2 to measure interval")
        print("FAIL")
        return 1

    intervals_ms = [(times[i] - times[i - 1]) * 1000.0 for i in range(1, len(times))]
    ordered = sorted(intervals_ms)
    mean = statistics.mean(intervals_ms)
    med = statistics.median(intervals_ms)
    p95 = percentile(ordered, 0.95)
    err = mean - args.expect_ms
    err_pct = (mean / args.expect_ms - 1.0) * 100.0 if args.expect_ms else float("nan")

    print(f"messages={len(times)}  intervals={len(intervals_ms)}")
    print(
        f"min={min(intervals_ms):.1f}  max={max(intervals_ms):.1f}  "
        f"mean={mean:.1f}  median={med:.1f}  p95={p95:.1f}  ms"
    )
    print(f"error vs expect: {err:+.1f} ms ({err_pct:+.1f}%)")
    if args.show > 0:
        shown = ", ".join(f"{x:.1f}" for x in intervals_ms[: args.show])
        print(f"first intervals (ms): {shown}")
    uniq = sorted(set(payloads))
    if len(uniq) <= 5:
        print(f"payloads seen: {uniq}")
    else:
        print(f"payloads: {len(uniq)} unique values")

    ok = abs(err) <= args.tolerance_ms
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

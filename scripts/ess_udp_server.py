#!/usr/bin/env python3
"""UDP server for ESS load2 power packets from the ESP32.

Packet format (ASCII):
  <power_w> <seq> <device_ms>

Example:
  ./scripts/ess_udp_server.py
  ./scripts/ess_udp_server.py --port 9999 --seconds 15 --expect-ms 250
"""

from __future__ import annotations

import argparse
import re
import socket
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SECRETS_H = ROOT / "include" / "secrets.h"
CONFIG_H = ROOT / "include" / "config.h"

PACKET_RE = re.compile(
    r"^\s*([+-]?(?:\d+\.?\d*|\.\d+))\s+(\d+)\s+(\d+)\s*$"
)


def _define_int(text: str, name: str, default: int) -> int:
    m = re.search(rf"#define\s+{name}\s+(\d+)", text)
    return int(m.group(1)) if m else default


def load_defaults() -> dict:
    port = 9999
    expect_ms = 250.0
    if SECRETS_H.is_file():
        port = _define_int(SECRETS_H.read_text(encoding="utf-8", errors="replace"), "ESS_UDP_PORT", port)
    if CONFIG_H.is_file():
        expect_ms = float(
            _define_int(CONFIG_H.read_text(encoding="utf-8", errors="replace"), "ESS_PUBLISH_INTERVAL_MS", 250)
        )
    return {"port": port, "expect_ms": expect_ms}


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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0", help="Bind address")
    ap.add_argument("--port", type=int, default=defaults["port"])
    ap.add_argument("--seconds", type=float, default=0.0, help="Exit after N seconds (0 = run forever)")
    ap.add_argument("--expect-ms", type=float, default=defaults["expect_ms"])
    ap.add_argument("--tolerance-ms", type=float, default=50.0)
    ap.add_argument("--report-every", type=int, default=20, help="Print stats every N packets")
    ap.add_argument("--quiet", action="store_true", help="Only print periodic stats / final report")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.settimeout(0.5)

    print(f"listening udp://{args.host}:{args.port}  expect={args.expect_ms:.1f} ms")
    print("waiting for packets from ESP (format: '<power> <seq> <millis>') ...")

    host_times: list[float] = []
    seqs: list[int] = []
    powers: list[float] = []
    start = time.monotonic()
    last_report = 0

    try:
        while True:
            if args.seconds > 0 and (time.monotonic() - start) >= args.seconds:
                break
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue

            text = data.decode("utf-8", errors="replace").strip()
            m = PACKET_RE.match(text)
            now = time.monotonic()
            if not m:
                print(f"from {addr[0]}: bad packet {text!r}")
                continue

            power = float(m.group(1))
            seq = int(m.group(2))
            device_ms = int(m.group(3))
            host_times.append(now)
            seqs.append(seq)
            powers.append(power)

            if not args.quiet:
                dt = 0.0 if len(host_times) < 2 else (host_times[-1] - host_times[-2]) * 1000.0
                print(f"{addr[0]}  P={power:.3f} W  seq={seq}  device_ms={device_ms}  host_dt={dt:.1f} ms")

            if args.report_every > 0 and len(host_times) - last_report >= args.report_every:
                last_report = len(host_times)
                _print_stats(host_times, seqs, args.expect_ms, args.tolerance_ms, interim=True)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        sock.close()

    if len(host_times) < 2:
        print(f"packets={len(host_times)} — need at least 2")
        print("FAIL")
        return 1

    ok = _print_stats(host_times, seqs, args.expect_ms, args.tolerance_ms, interim=False)
    return 0 if ok else 1


def _print_stats(
    host_times: list[float],
    seqs: list[int],
    expect_ms: float,
    tolerance_ms: float,
    interim: bool,
) -> bool:
    intervals = [(host_times[i] - host_times[i - 1]) * 1000.0 for i in range(1, len(host_times))]
    sample = intervals[1:] if len(intervals) > 3 else intervals
    if not sample:
        print("not enough intervals yet")
        return False

    ordered = sorted(sample)
    mean = statistics.mean(sample)
    med = statistics.median(sample)
    p95 = percentile(ordered, 0.95)
    err = mean - expect_ms

    gaps = 0
    for i in range(1, len(seqs)):
        step = seqs[i] - seqs[i - 1]
        if step > 1:
            gaps += step - 1
        elif step <= 0:
            gaps += 1

    label = "interim" if interim else "final"
    print(
        f"[{label}] packets={len(host_times)} intervals={len(sample)} "
        f"min={min(sample):.1f} mean={mean:.1f} median={med:.1f} p95={p95:.1f} ms "
        f"err={err:+.1f} seq_gaps={gaps}"
    )
    ok = abs(err) <= tolerance_ms and gaps == 0
    if not interim:
        print("PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(main())

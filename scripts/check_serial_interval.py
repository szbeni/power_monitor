#!/usr/bin/env python3
"""Measure [ess] serial tick interval from the device USB console.

Example:
  ./scripts/check_serial_interval.py
  ./scripts/check_serial_interval.py --port /dev/ttyACM0 --seconds 10 --expect-ms 250
"""

from __future__ import annotations

import argparse
import re
import statistics
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pip install pyserial  (or: python3 -m pip install --break-system-packages pyserial)",
          file=sys.stderr)
    sys.exit(1)

LINE_RE = re.compile(r"^\[ess\].*\bdt=(\d+)\s*ms\b")


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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--expect-ms", type=float, default=250.0)
    ap.add_argument("--tolerance-ms", type=float, default=40.0)
    ap.add_argument("--show", type=int, default=12)
    args = ap.parse_args()

    print(f"port={args.port} baud={args.baud} expect={args.expect_ms:.1f} ms")
    print(f"sampling for {args.seconds:.1f}s ...")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"open failed: {exc}", file=sys.stderr)
        return 2

    # Opening the port often resets ESP32-C3 USB-JTAG — wait for firmware to boot.
    time.sleep(2.0)
    ser.reset_input_buffer()

    host_times: list[float] = []
    device_dts: list[float] = []
    deadline = time.monotonic() + args.seconds
    buf = ""

    try:
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk.decode("utf-8", errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                m = LINE_RE.match(line)
                if not m:
                    continue
                host_times.append(time.monotonic())
                # Skip first device dt (0 or boot gap)
                dt = float(m.group(1))
                if len(host_times) > 1 and dt > 0:
                    device_dts.append(dt)
    finally:
        ser.close()

    if len(host_times) < 3:
        print(f"ess_lines={len(host_times)} — need more [ess] lines (is firmware flashing/running?)")
        print("FAIL")
        return 1

    host_intervals = [(host_times[i] - host_times[i - 1]) * 1000.0 for i in range(1, len(host_times))]
    # drop first host interval (often a catch-up after open)
    sample = host_intervals[1:] if len(host_intervals) > 3 else host_intervals
    ordered = sorted(sample)
    mean = statistics.mean(sample)
    med = statistics.median(sample)
    p95 = percentile(ordered, 0.95)
    err = mean - args.expect_ms

    print(f"ess_lines={len(host_times)}  host_intervals={len(sample)}")
    print(
        f"host timing ms: min={min(sample):.1f} max={max(sample):.1f} "
        f"mean={mean:.1f} median={med:.1f} p95={p95:.1f}"
    )
    print(f"error vs expect: {err:+.1f} ms ({(mean / args.expect_ms - 1) * 100:+.1f}%)")
    if device_dts:
        print(
            f"device-reported dt ms: min={min(device_dts):.0f} max={max(device_dts):.0f} "
            f"mean={statistics.mean(device_dts):.1f} median={statistics.median(device_dts):.1f}"
        )
    if args.show > 0:
        print("first host intervals (ms):", ", ".join(f"{x:.1f}" for x in sample[: args.show]))

    ok = abs(err) <= args.tolerance_ms
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

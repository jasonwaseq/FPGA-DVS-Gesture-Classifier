#!/usr/bin/env python3
"""
Relay EVT2 stream to FPGA UART by converting EVT2 CD events into 5-byte packets.

Assumes EVT2 word format with type in bits [31:28] and CD events in types 0/1.
Defaults are configurable via CLI for bit positions and swapping X/Y.
"""

import argparse
import threading
import time
from collections import Counter

import serial

VALID_TYPES = {0, 1, 8, 0xA, 0xE, 0xF}


def detect_best_offset(data: bytes, sample_words: int = 20000):
    """Return (best_offset, valid_ratio, type_counts_for_best)."""
    best = (None, -1.0, None)
    for off in range(4):
        n = (len(data) - off) // 4
        if n <= 0:
            continue
        n = min(n, sample_words)
        counts = Counter()
        valid = 0
        total = 0
        base = off
        for i in range(n):
            w = int.from_bytes(data[base + 4 * i : base + 4 * i + 4], "little")
            t = (w >> 28) & 0xF
            counts[t] += 1
            total += 1
            if t in VALID_TYPES:
                valid += 1
        ratio = valid / total if total else 0.0
        if ratio > best[1]:
            best = (off, ratio, counts)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in-port", required=True, help="Input EVT2 port, e.g. /dev/ttyACM0")
    parser.add_argument("--out-port", required=True, help="FPGA UART port, e.g. /dev/ttyUSB1")
    parser.add_argument("--in-baud", type=int, default=115200)
    parser.add_argument("--out-baud", type=int, default=115200)
    parser.add_argument("--chunk", type=int, default=4096)
    parser.add_argument("--swap-xy", action="store_true", default=True,
                        help="Swap decoded X/Y (default: on)")
    parser.add_argument("--no-swap-xy", dest="swap_xy", action="store_false")
    parser.add_argument("--x-shift", type=int, default=17)
    parser.add_argument("--y-shift", type=int, default=6)
    parser.add_argument("--x-mask", type=lambda v: int(v, 0), default=0x7FF)
    parser.add_argument("--y-mask", type=lambda v: int(v, 0), default=0x7FF)
    parser.add_argument("--pol-bit", type=int, default=5)
    parser.add_argument("--type-shift", type=int, default=28)
    parser.add_argument("--type-mask", type=lambda v: int(v, 0), default=0xF)
    args = parser.parse_args()

    print(f"Opening EVT2 input: {args.in_port} @ {args.in_baud}")
    sin = serial.Serial(args.in_port, args.in_baud, timeout=0.1)
    print(f"Opening FPGA output: {args.out_port} @ {args.out_baud}")
    sout = serial.Serial(args.out_port, args.out_baud, timeout=0.1)

    stop = False
    stats = {
        "words": 0,
        "cd_events": 0,
        "dropped": 0,
        "gesture": 0,
        "last_report": time.time(),
    }

    def read_fpga():
        while not stop:
            try:
                if sout.in_waiting:
                    data = sout.read(sout.in_waiting)
                    for b in data:
                        if (b & 0xF0) == 0xA0:
                            stats["gesture"] += 1
                            g = b & 0x03
                            name = ["UP", "DOWN", "LEFT", "RIGHT"][g]
                            print(f"\nFPGA GESTURE: {name}")
            except Exception:
                pass
            time.sleep(0.01)

    t = threading.Thread(target=read_fpga, daemon=True)
    t.start()

    # Initial alignment probe
    probe = sin.read(args.chunk)
    if len(probe) >= 16:
        off, ratio, counts = detect_best_offset(probe)
        if off is not None:
            print(f"Alignment offset: {off} (valid ratio {ratio:.3f})")
            probe = probe[off:]
            if counts:
                top = ", ".join(f"{k}:{v}" for k, v in counts.most_common(4))
                print(f"Type counts: {top}")
    buf = bytearray(probe)

    try:
        while True:
            data = sin.read(args.chunk)
            if data:
                buf.extend(data)

            # Process complete words
            n = (len(buf) // 4) * 4
            if n == 0:
                continue

            for i in range(0, n, 4):
                w = int.from_bytes(buf[i:i+4], "little")
                stats["words"] += 1
                tval = (w >> args.type_shift) & args.type_mask
                if tval == 0 or tval == 1:
                    x = (w >> args.x_shift) & args.x_mask
                    y = (w >> args.y_shift) & args.y_mask
                    if args.swap_xy:
                        x, y = y, x
                    pol = 1 if tval == 1 else 0
                    if x < 320 and y < 320:
                        pkt = bytes([(x >> 8) & 0x01, x & 0xFF, (y >> 8) & 0x01, y & 0xFF, pol])
                        sout.write(pkt)
                        stats["cd_events"] += 1
                    else:
                        stats["dropped"] += 1

            # Keep remainder for next read
            buf = buf[n:]

            now = time.time()
            if now - stats["last_report"] >= 1.0:
                print(
                    f"words={stats['words']} cd={stats['cd_events']} "
                    f"drop={stats['dropped']} gest={stats['gesture']}"
                )
                stats["last_report"] = now

    except KeyboardInterrupt:
        pass
    finally:
        stop = True
        sin.close()
        sout.close()


if __name__ == "__main__":
    main()

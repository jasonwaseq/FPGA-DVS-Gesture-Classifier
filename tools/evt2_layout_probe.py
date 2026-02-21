#!/usr/bin/env python3
"""Probe EVT2 bit-field layout from GenX320 by testing candidate X/Y/pol arrangements."""

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


VALID_TYPES = {0, 1, 8, 0xA, 0xE, 0xF}
CD_TYPES = {0, 1}

# (label, x_shift, x_bits, y_shift, y_bits, swap_xy); type always in bits [31:28]
LAYOUTS = [
    ("Standard EVT2  (x[21:11] y[10:0])",               11, 11,  0, 11, False),
    ("Standard swapped (x[10:0] y[21:11])",               0, 11, 11, 11, False),
    ("Relay default  (raw_x[27:17] raw_y[16:6] swap)",   17, 11,  6, 11, True),
    ("Relay no-swap  (x[27:17] y[16:6])",                17, 11,  6, 11, False),
    ("9-bit A  (x[19:11] y[10:2])",                      11,  9,  2,  9, False),
    ("9-bit B  (x[10:2]  y[19:11])",                      2,  9, 11,  9, False),
    ("Pol[0] x[10:1] y[20:11]",                           1, 10, 11, 10, False),
    ("Pol[0] y[10:1] x[20:11]",                          11, 10,  1, 10, False),
    ("Pol[0] x[9:1]  y[18:10]",                           1,  9, 10,  9, False),
    ("Pol[0] y[9:1]  x[18:10]",                          10,  9,  1,  9, False),
]


def make_mask(bits):
    return (1 << bits) - 1


# ── Analysis ────────────────────────────────────────────────────────────────

def analyse_layout(words, label, x_shift, x_bits, y_shift, y_bits, swap_xy):
    x_mask = make_mask(x_bits)
    y_mask = make_mask(y_bits)
    xs, ys = [], []
    oob = 0
    for w in words:
        t = (w >> 28) & 0xF
        if t not in CD_TYPES:
            continue
        raw_x = (w >> x_shift) & x_mask
        raw_y = (w >> y_shift) & y_mask
        if swap_xy:
            raw_x, raw_y = raw_y, raw_x
        if raw_x > 319 or raw_y > 319:
            oob += 1
            continue
        xs.append(raw_x)
        ys.append(raw_y)
    n = len(xs)
    if n == 0:
        return None
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    gx_vals = [min(v >> 5, 15) for v in xs]
    gy_vals = [min(v >> 5, 15) for v in ys]
    gx_mean = sum(gx_vals) / n
    gy_mean = sum(gy_vals) / n
    x_bins = [0] * 10
    y_bins = [0] * 10
    for v in xs:
        x_bins[min(v // 32, 9)] += 1
    for v in ys:
        y_bins[min(v // 32, 9)] += 1

    def uniformity(bins):
        m = sum(bins) / len(bins)
        if m == 0:
            return 0.0
        var = sum((b - m) ** 2 for b in bins) / len(bins)
        return (var ** 0.5) / m

    return {
        "label": label,
        "n": n,
        "oob": oob,
        "x_min": x_min, "x_max": x_max, "x_mean": x_mean,
        "y_min": y_min, "y_max": y_max, "y_mean": y_mean,
        "gx_mean": gx_mean, "gy_mean": gy_mean,
        "x_bins": x_bins, "y_bins": y_bins,
        "x_cv": uniformity(x_bins),
        "y_cv": uniformity(y_bins),
        "x_span": x_max - x_min,
        "y_span": y_max - y_min,
    }


def print_report(stats):
    s = stats
    print(f"\n{'─'*74}")
    print(f"  {s['label']}")
    print(f"{'─'*74}")
    print(f"  CD events decoded: {s['n']:,}   out-of-range (>319): {s['oob']:,}")
    print(f"  X range: [{s['x_min']:3d}, {s['x_max']:3d}]  mean={s['x_mean']:.1f}  span={s['x_span']}")
    print(f"  Y range: [{s['y_min']:3d}, {s['y_max']:3d}]  mean={s['y_mean']:.1f}  span={s['y_span']}")
    print(f"  Grid centroid: ({s['gx_mean']:.2f}, {s['gy_mean']:.2f})  offset=({s['gx_mean']-4.5:+.2f}, {s['gy_mean']-4.5:+.2f})")
    print(f"\n  X distribution (32-px bins):  CV={s['x_cv']:.3f}")
    mx = max(s["x_bins"]) or 1
    for i, c in enumerate(s["x_bins"]):
        lo, hi = i * 32, i * 32 + 31
        print(f"    [{lo:3d}-{hi:3d}]: {c:6d} {'#' * int(40 * c / mx)}")
    print(f"\n  Y distribution (32-px bins):  CV={s['y_cv']:.3f}")
    mx = max(s["y_bins"]) or 1
    for i, c in enumerate(s["y_bins"]):
        lo, hi = i * 32, i * 32 + 31
        print(f"    [{lo:3d}-{hi:3d}]: {c:6d} {'#' * int(40 * c / mx)}")


def score_layout(stats):
    """Higher score = more likely correct layout."""
    if stats is None:
        return -1e9
    s = stats
    score = 0.0
    total = s["n"] + s["oob"]
    if total == 0:
        return -1e9
    score -= (s["oob"] / total) * 1000
    score += (s["x_span"] / 319) * 100
    score += (s["y_span"] / 319) * 100
    score -= s["x_cv"] * 30
    score -= s["y_cv"] * 30
    score -= abs(s["x_mean"] - 160) * 0.3
    score -= abs(s["y_mean"] - 160) * 0.3
    return score


def capture_from_port(port_name, baud, duration):
    if serial is None:
        print("ERROR: pyserial is required.  pip install pyserial")
        sys.exit(1)

    print(f"Opening {port_name} @ {baud}...")
    ser = serial.Serial(port_name, baud, timeout=0.5)
    print(f"Capturing for {duration:.1f}s — move your hand in all four directions")
    print(f"over the sensor to exercise the full coordinate range.\n")

    raw = bytearray()
    start = time.time()
    while time.time() - start < duration:
        chunk = ser.read(4096)
        if chunk:
            raw.extend(chunk)
            mb = len(raw) / (1024 * 1024)
            sys.stdout.write(f"\r  {mb:.2f} MB captured")
            sys.stdout.flush()

    ser.close()
    print(f"\n  Total: {len(raw):,} bytes\n")
    return bytes(raw)


def load_from_file(path):
    data = Path(path).read_bytes()
    print(f"Loaded {len(data):,} bytes from {path}\n")
    return data


def align_and_parse(raw):
    """Find the best 4-byte alignment and return list of uint32 words."""
    best_off, best_ratio = 0, -1.0
    for off in range(4):
        n = (len(raw) - off) // 4
        if n < 100:
            continue
        sample = min(n, 50000)
        valid = 0
        for i in range(sample):
            w = int.from_bytes(raw[off + 4 * i: off + 4 * i + 4], "little")
            if ((w >> 28) & 0xF) in VALID_TYPES:
                valid += 1
        ratio = valid / sample
        if ratio > best_ratio:
            best_off, best_ratio = off, ratio

    aligned = raw[best_off:]
    aligned = aligned[: (len(aligned) // 4) * 4]
    n_words = len(aligned) // 4
    words = [int.from_bytes(aligned[4 * i: 4 * i + 4], "little") for i in range(n_words)]
    type_counts = Counter((w >> 28) & 0xF for w in words)
    cd_count = sum(type_counts.get(t, 0) for t in CD_TYPES)
    print(f"Alignment: offset={best_off}  valid-type ratio={best_ratio:.4f}")
    print(f"Words: {n_words:,}   CD events: {cd_count:,}")
    print(f"Type distribution: {dict(type_counts.most_common())}")
    print(f"\nFirst 8 raw words (hex, little-endian):")
    for i in range(min(8, n_words)):
        w = words[i]
        t = (w >> 28) & 0xF
        print(f"  [{i}] 0x{w:08X}  type={t:X}  bits[27:22]={((w>>22)&0x3F):06b}  bits[21:11]={(w>>11)&0x7FF:4d}  bits[10:0]={w&0x7FF:4d}")
    return words


def main():
    ap = argparse.ArgumentParser(description="Probe EVT2 bit layout from GenX320")
    ap.add_argument("port", nargs="?", help="Serial port (e.g. /dev/ttyACM0, COM5)")
    ap.add_argument("--file", "-f", help="Analyse a saved .bin file instead of live capture")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=float, default=3.0,
                    help="Capture duration in seconds (default: 3)")
    args = ap.parse_args()

    if not args.port and not args.file:
        ap.print_help()
        print("\nERROR: provide a serial port or --file")
        return 1
    if args.file:
        raw = load_from_file(args.file)
    else:
        raw = capture_from_port(args.port, args.baud, args.duration)

    if len(raw) < 100:
        print("ERROR: not enough data captured")
        return 1

    words = align_and_parse(raw)
    if not words:
        return 1
    print(f"\n{'='*74}")
    print(f"  TESTING {len(LAYOUTS)} CANDIDATE LAYOUTS")
    print(f"{'='*74}")
    results = []
    for label, x_shift, x_bits, y_shift, y_bits, swap in LAYOUTS:
        stats = analyse_layout(words, label, x_shift, x_bits, y_shift, y_bits, swap)
        if stats is not None:
            print_report(stats)
            results.append((score_layout(stats), stats))
    results.sort(key=lambda r: r[0], reverse=True)
    print(f"\n{'='*74}")
    print(f"  RANKING (best first)")
    print(f"{'='*74}")
    for rank, (sc, s) in enumerate(results, 1):
        oob_pct = 100 * s["oob"] / (s["n"] + s["oob"]) if (s["n"] + s["oob"]) else 0
        flag = "  <<<  BEST" if rank == 1 else ""
        print(f"  {rank}. {s['label']:<50s}  score={sc:+7.1f}  span=({s['x_span']:3d},{s['y_span']:3d})  oob={oob_pct:.0f}%{flag}")
    best_stats = results[0][1]
    print(f"\n{'='*74}")
    print(f"  RECOMMENDED LAYOUT: {best_stats['label']}")
    print(f"  Grid centroid offset: ({best_stats['gx_mean']-4.5:+.2f}, {best_stats['gy_mean']-4.5:+.2f})")
    print(f"{'='*74}")
    best_label = best_stats["label"]
    for label, x_shift, x_bits, y_shift, y_bits, swap in LAYOUTS:
        if label == best_label:
            swap_flag = " --swap-xy" if swap else ""
            print(f"  replay_evt_to_fpga.py flags:")
            print(f"    --x-shift {x_shift} --x-mask 0x{make_mask(x_bits):X} --y-shift {y_shift} --y-mask 0x{make_mask(y_bits):X}{swap_flag}")
            break
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())

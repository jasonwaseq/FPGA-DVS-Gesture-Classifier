"""
EVT2.0 .bin file diagnostic script for gesture classifier analysis.
Analyzes spatial heatmaps, dominant columns, and weight-score simulation.
"""

import struct
import os
import sys

# Force UTF-8 stdout to avoid cp1252 issues on Windows
sys.stdout.reconfigure(encoding='utf-8')

# -- Paths --------------------------------------------------------------------
BASE = r"C:\Users\jason\Documents\FPGA-DVS-Gesture-Classifier"
GESTURE_FILES = {
    "left":  os.path.join(BASE, "tools", "gesture_left.bin"),
    "right": os.path.join(BASE, "tools", "gesture_right.bin"),
    "down":  os.path.join(BASE, "tools", "gesture_down.bin"),
    "up":    os.path.join(BASE, "tools", "gesture_up.bin"),
}
WEIGHT_DIR = os.path.join(BASE, "weights")
CLASS_NAMES = ["Down", "Left", "Right", "Up"]

# -- EVT2 decoder -------------------------------------------------------------
TYPE_CD_OFF    = 0x0
TYPE_CD_ON     = 0x1
TYPE_TIME_HIGH = 0x8

SENSOR_SIZE = 320
GRID_SIZE   = 8
CELL_SIZE   = 40   # 320 / 8 = 40 pixels per cell

def decode_evt2_file(path):
    """Decode EVT2.0 binary. Returns list of (full_ts, sensor_x, sensor_y)."""
    events = []
    time_high = 0
    with open(path, "rb") as f:
        data = f.read()
    n_words = len(data) // 4
    for i in range(n_words):
        word = struct.unpack_from("<I", data, i * 4)[0]
        evt_type = (word >> 28) & 0xF
        if evt_type == TYPE_TIME_HIGH:
            time_high = word & 0x0FFFFFFF  # bits[27:0]
        elif evt_type in (TYPE_CD_OFF, TYPE_CD_ON):
            ts_lsb = (word >> 22) & 0x3F
            x = (word >> 11) & 0x7FF
            y = word & 0x7FF
            full_ts = (time_high << 6) | ts_lsb
            events.append((full_ts, x, y))
    return events

def apply_transforms(sensor_x, sensor_y):
    """Apply swap_xy then flip_x. Returns (fpga_x, fpga_y)."""
    new_x = sensor_y   # swap
    new_y = sensor_x
    fpga_x = 319 - new_x  # flip_x
    fpga_y = new_y
    return fpga_x, fpga_y

def to_grid(fpga_x, fpga_y):
    """Convert FPGA pixel coords to 8x8 grid cell."""
    cell_x = min(fpga_x // CELL_SIZE, GRID_SIZE - 1)
    cell_y = min(fpga_y // CELL_SIZE, GRID_SIZE - 1)
    return cell_x, cell_y

def normalize_val(v, max_v, scale=9):
    if max_v == 0:
        return 0
    return int(v * scale / max_v)

# -- Weight loading -----------------------------------------------------------
def load_weights(class_idx):
    """Load 256 weights for a class from 256weights_q8_c{N}.mem."""
    path = os.path.join(WEIGHT_DIR, f"256weights_q8_c{class_idx}.mem")
    with open(path) as f:
        lines = [l.strip() for l in f if l.strip()]
    weights = [int(h, 16) for h in lines]
    assert len(weights) == 256, f"Expected 256 weights, got {len(weights)}"
    return weights

# -- Rate limiting ------------------------------------------------------------
def rate_limit_events(events, rate=2000):
    """Keep events spaced >= 1/rate seconds apart (token bucket)."""
    if not events:
        return []
    min_gap_us = 1_000_000 / rate  # EVT2 timestamps are in microseconds
    kept = [events[0]]
    last_ts = events[0][0]
    for ev in events[1:]:
        ts, sx, sy = ev
        if (ts - last_ts) >= min_gap_us:
            kept.append(ev)
            last_ts = ts
    return kept

# -- Per-file analysis --------------------------------------------------------
def analyze_file(gesture_name, path, all_weights):
    print("=" * 72)
    print(f"FILE: {os.path.basename(path)}  (expected: {gesture_name.upper()})")
    print("=" * 72)

    raw_events = decode_evt2_file(path)
    if not raw_events:
        print("  [ERROR] No events decoded!")
        return

    print(f"  Total CD events: {len(raw_events):,}")

    # Transform to FPGA coordinates
    fpga_events = []
    for ts, sx, sy in raw_events:
        fx, fy = apply_transforms(sx, sy)
        cx, cy = to_grid(fx, fy)
        fpga_events.append((ts, fx, fy, cx, cy))

    ts_min = fpga_events[0][0]
    ts_max = fpga_events[-1][0]
    duration_us = ts_max - ts_min
    duration_ms = duration_us / 1000
    print(f"  Time span: {ts_min} - {ts_max} us  ({duration_ms:.1f} ms)")

    # -- Part 1: Spatial heatmap per time slice --------------------------------
    print()
    print("-- Part 1: Spatial heatmap --")
    print("   Rows=time slices 0->7 (early->late), Cols=FPGA cell_x 0->7")
    print("   Cell value = normalized event count (0=none, 9=max)")
    print()

    N_SLICES = 8
    # slice_counts[slice][cell_x] summed over all cell_y
    slice_counts = [[0] * GRID_SIZE for _ in range(N_SLICES)]

    if duration_us > 0:
        for ts, fx, fy, cx, cy in fpga_events:
            s = min(int((ts - ts_min) * N_SLICES / duration_us), N_SLICES - 1)
            slice_counts[s][cx] += 1
    else:
        for ts, fx, fy, cx, cy in fpga_events:
            slice_counts[0][cx] += 1

    global_max = max(max(row) for row in slice_counts)

    print("        col_x:  0   1   2   3   4   5   6   7     total")
    print("                -   -   -   -   -   -   -   -")
    for s in range(N_SLICES):
        row_str = "   ".join(str(normalize_val(slice_counts[s][cx], global_max))
                             for cx in range(GRID_SIZE))
        total = sum(slice_counts[s])
        print(f"   t={s}:        {row_str}    ({total:6,})")

    # -- Part 2: Dominant x-column per time slice -----------------------------
    print()
    print("-- Part 2: Dominant cell_x per time slice --")
    print(f"   {'Slice':<6}  {'Total events':>13}  {'Dom cell_x':>10}  {'Dom count':>10}")
    for s in range(N_SLICES):
        total = sum(slice_counts[s])
        dom_cx = slice_counts[s].index(max(slice_counts[s]))
        dom_cnt = slice_counts[s][dom_cx]
        print(f"   t={s}:   {total:13,}  {dom_cx:10d}  {dom_cnt:10,}")

    # -- Part 3: Weight-score simulation --------------------------------------
    print()
    print("-- Part 3: FPGA weight-score simulation (2000 ev/s rate limit) --")

    rl_events = rate_limit_events(raw_events, rate=2000)
    pct = len(rl_events) / len(raw_events) * 100
    print(f"   Events after rate-limiting: {len(rl_events):,}  ({pct:.1f}% of {len(raw_events):,})")

    # Transform rate-limited events to FPGA grid
    rl_fpga = []
    for ts, sx, sy in rl_events:
        fx, fy = apply_transforms(sx, sy)
        cx, cy = to_grid(fx, fy)
        rl_fpga.append((ts, cx, cy))

    # Bin into 4 x 250ms bins
    BIN_DURATION_US = 250_000
    N_BINS = 4

    rl_ts_min = rl_fpga[0][0] if rl_fpga else 0

    # feature_counts[bin][cell_y*8 + cell_x], saturated at 15
    feature_counts = [[0] * 64 for _ in range(N_BINS)]
    bin_event_counts = [0] * N_BINS
    events_beyond = 0

    for ts, cx, cy in rl_fpga:
        rel_us = ts - rl_ts_min
        b = int(rel_us // BIN_DURATION_US)
        if b >= N_BINS:
            events_beyond += 1
            continue
        fi = cy * GRID_SIZE + cx
        feature_counts[b][fi] = min(feature_counts[b][fi] + 1, 15)
        bin_event_counts[b] += 1

    print(f"   Events beyond 1s window (dropped): {events_beyond:,}")
    print(f"   Events per FPGA bin: {bin_event_counts}")

    print()
    print("   Feature vector: top 5 active cells per bin")
    for b in range(N_BINS):
        active = sorted([(feature_counts[b][i], i) for i in range(64)
                         if feature_counts[b][i] > 0], reverse=True)
        top5 = active[:5]
        if top5:
            s = ", ".join(f"(cx={i%8},cy={i//8})={v}" for v, i in top5)
        else:
            s = "(empty)"
        print(f"   bin{b}: {s}")

    # Compute scores
    scores = [0] * 4
    for c in range(4):
        w = all_weights[c]
        for b in range(N_BINS):
            for fi in range(64):
                wi = b * 64 + fi
                scores[c] += feature_counts[b][fi] * w[wi]

    print()
    print("   Scores (raw, unscaled):")
    max_score = max(scores)
    for c in range(4):
        marker = " <-- WINNER" if scores[c] == max_score else ""
        print(f"   Class {c} ({CLASS_NAMES[c]:<5}): {scores[c]:8d}{marker}")

    winner = scores.index(max_score)
    correct = "CORRECT" if CLASS_NAMES[winner].lower() == gesture_name else "WRONG"
    print(f"   -> Classified as: {CLASS_NAMES[winner]}  [{correct}]  (expected: {gesture_name.upper()})")
    print()

# -- Weight analysis ----------------------------------------------------------
def analyze_weights(all_weights):
    print("=" * 72)
    print("WEIGHT FILE ANALYSIS")
    print("=" * 72)

    print()
    print("-- Weight sums per class (raw / 1024 to verify ratio) --")
    for c in range(4):
        w = all_weights[c]
        total_raw = sum(w)
        total_norm = total_raw / 1024.0
        print(f"  Class {c} ({CLASS_NAMES[c]:<5}): raw_sum={total_raw:6d}  /1024={total_norm:.2f}")

    print()
    print("-- Per-bin weight sums per class --")
    print(f"  {'Class':<8}   bin0     bin1     bin2     bin3")
    for c in range(4):
        w = all_weights[c]
        bin_sums = [sum(w[b*64:(b+1)*64]) / 1024.0 for b in range(4)]
        vals = "  ".join(f"{v:7.2f}" for v in bin_sums)
        print(f"  {CLASS_NAMES[c]:<8}   {vals}")

    # Top 10 features for Left and Right
    for target_class in [1, 2]:
        name = CLASS_NAMES[target_class]
        w = all_weights[target_class]
        indexed = sorted(enumerate(w), key=lambda x: x[1], reverse=True)
        print()
        print(f"-- Top 10 highest-weight features for {name} --")
        print(f"  {'Rank':<5}  {'feat_idx':<9}  {'bin':<4}  {'cell_y':<7}  {'cell_x':<7}  {'weight':<7}")
        for rank, (fi, wt) in enumerate(indexed[:10], 1):
            b = fi // 64
            cell = fi % 64
            cy = cell // 8
            cx = cell % 8
            print(f"  {rank:<5}  {fi:<9}  {b:<4}  {cy:<7}  {cx:<7}  {wt}")

    # Left minus Right weight difference grid
    print()
    print("-- Left vs Right weight diff per cell (Left-Right, summed over bins) --")
    print("   Positive=Left>Right, Negative=Right>Left")
    wL = all_weights[1]
    wR = all_weights[2]
    cell_diff = []
    for cy in range(8):
        row = []
        for cx in range(8):
            diff = sum(wL[b*64 + cy*8 + cx] - wR[b*64 + cy*8 + cx] for b in range(4))
            row.append(diff)
        cell_diff.append(row)

    header = "  cy\\cx   " + "  ".join(f"{cx:4d}" for cx in range(8))
    print(f"  {header}")
    for cy in range(8):
        vals = "  ".join(f"{cell_diff[cy][cx]:+4d}" for cx in range(8))
        print(f"  cy={cy}:    {vals}")

    # Per-bin, per-cx sums for Left and Right
    print()
    print("-- Left vs Right: per-bin per-cx weight sums (summed over all cy=0..7) --")
    for target_class in [1, 2]:
        w = all_weights[target_class]
        name = CLASS_NAMES[target_class]
        print()
        print(f"  {name}:")
        print("  " + "  ".join(f"  cx={cx}" for cx in range(8)))
        for b in range(4):
            vals = [sum(w[b*64 + cy*8 + cx] for cy in range(8)) for cx in range(8)]
            val_str = "  ".join(f"{v:5d}" for v in vals)
            print(f"  bin{b}:  {val_str}")

    print()

# -- Main ---------------------------------------------------------------------
def main():
    print()
    print("EVT2.0 Gesture File Diagnostic")
    print("Transforms: swap_xy then flip_x, then grid cell = px // 40")
    print(f"Weight files: 256weights_q8_c0-3.mem in {WEIGHT_DIR}")
    print()

    all_weights = []
    for c in range(4):
        all_weights.append(load_weights(c))
    print(f"Loaded weight files: {CLASS_NAMES}")
    print()

    analyze_weights(all_weights)

    for gesture_name, path in GESTURE_FILES.items():
        if not os.path.exists(path):
            print(f"[SKIP] Not found: {path}")
            continue
        analyze_file(gesture_name, path, all_weights)

    print("Done.")

if __name__ == "__main__":
    main()
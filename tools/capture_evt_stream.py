#!/usr/bin/env python3
# usage: python3 capture_evt_stream.py COM3 trial_run.bin

import argparse
import time
import threading
from collections import Counter
import serial

stop_flag = False

VALID_TYPES = {0, 1, 8, 0xA, 0xE, 0xF}  # per evt_2_0.h


def wait_for_key():
    global stop_flag
    input("\nPress ENTER to stop capture...\n")
    stop_flag = True


def detect_best_offset(data: bytes, sample_words: int = 50000):
    """
    Returns (best_offset, valid_ratio, type_counts_for_best).
    Uses EVT2 type = (word >> 28) & 0xF, little-endian words.
    """
    best = (None, -1.0, None)

    for off in range(4):
        n = (len(data) - off) // 4
        if n <= 0:
            continue
        n = min(n, sample_words)

        counts = Counter()
        valid = 0
        total = 0

        # scan first n words
        base = off
        for i in range(n):
            w = int.from_bytes(data[base + 4 * i : base + 4 * i + 4], "little", signed=False)
            t = (w >> 28) & 0xF
            counts[t] += 1
            total += 1
            if t in VALID_TYPES:
                valid += 1

        ratio = valid / total if total else 0.0
        if ratio > best[1]:
            best = (off, ratio, counts)

    return best


def write_aligned(raw_path: str, aligned_path: str = "aligned.bin", sample_words: int = 50000):
    raw = open(raw_path, "rb").read()
    off, ratio, counts = detect_best_offset(raw, sample_words=sample_words)

    if off is None:
        raise RuntimeError("Could not determine alignment (file too small).")

    # Trim to 32-bit boundary from chosen offset
    aligned = raw[off:]
    aligned = aligned[: (len(aligned) // 4) * 4]

    with open(aligned_path, "wb") as f:
        f.write(aligned)

    return off, ratio, counts, len(raw), len(aligned)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="e.g. COM3")
    parser.add_argument("outfile", help="Raw output file, e.g. trial_run.bin")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Ignored by USB CDC but required by pyserial")
    parser.add_argument("--chunk", type=int, default=4096)
    parser.add_argument("--aligned", default="aligned.bin",
                        help="Aligned output file (default: aligned.bin)")
    parser.add_argument("--sample-words", type=int, default=50000,
                        help="Words to sample for offset detection (default: 50000)")
    args = parser.parse_args()

    print("\nOpening port:", args.port)
    ser = serial.Serial(args.port, args.baud, timeout=1)

    input("\nPress ENTER to start capture...\n")
    print("Capturing... (press ENTER again to stop)")

    global stop_flag
    stop_flag = False
    t = threading.Thread(target=wait_for_key, daemon=True)
    t.start()

    total = 0
    start = time.time()
    last_print = start

    with open(args.outfile, "wb") as f:
        while not stop_flag:
            data = ser.read(args.chunk)
            if data:
                f.write(data)
                total += len(data)

            now = time.time()
            if now - last_print >= 1.0:
                elapsed = now - start
                mb = total / (1024 * 1024)
                rate = mb / elapsed if elapsed > 0 else 0
                print(f"{mb:.2f} MB captured  |  {rate:.2f} MB/s")
                last_print = now

    ser.close()

    print("\nCapture complete.")
    print("Raw bytes written:", total)
    print("Raw file:", args.outfile)

    # Auto-align
    off, ratio, counts, raw_len, aligned_len = write_aligned(
        args.outfile, aligned_path=args.aligned, sample_words=args.sample_words
    )

    print("\nAlignment results:")
    print(f"  Best offset: {off}")
    print(f"  Valid EVT2 type ratio (sample): {ratio:.3f}")
    print(f"  Raw size: {raw_len} bytes, Aligned size: {aligned_len} bytes")
    print(f"  Aligned file: {args.aligned}")
    print("  Type counts (top 10):")
    for k, v in counts.most_common(10):
        print(f"    type {k:>2}: {v}")


if __name__ == "__main__":
    main()

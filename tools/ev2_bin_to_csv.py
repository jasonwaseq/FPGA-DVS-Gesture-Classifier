#!/usr/bin/env python3
"""
Decode a raw EVT2.0 binary stream (.bin) into CSV (x,y,polarity,timestamp_us).

EVT2.0 (Prophesee) summary:
- Each word is 32 bits, with type in bits [31:28]
- CD_OFF type=0x0, CD_ON type=0x1:
    bits [27:22] : ts_low (6 bits)
    bits [21:11] : x (11 bits)
    bits [10:0]  : y (11 bits)
- EVT_TIME_HIGH type=0x8:
    bits [27:0] : ts_high (28 bits) representing timestamp bits [33:6]
Full timestamp (microseconds) = (ts_high << 6) | ts_low

Reference: Metavision SDK docs, EVT 2.0 format. :contentReference[oaicite:1]{index=1}
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import sys
from typing import BinaryIO, Optional


TYPE_CD_OFF = 0x0
TYPE_CD_ON = 0x1
TYPE_TIME_HIGH = 0x8

DEFAULT_X_MAX = 2047  # EVT2 supports up to 2048x2048 (11-bit x/y fields) :contentReference[oaicite:2]{index=2}
DEFAULT_Y_MAX = 2047


def iter_words(f: BinaryIO, endian: str, chunk_words: int = 262144):
    """
    Yield 32-bit words from file.
    endian: 'little' or 'big'
    """
    if endian not in ("little", "big"):
        raise ValueError("endian must be 'little' or 'big'")

    fmt_prefix = "<" if endian == "little" else ">"
    # We'll unpack in batches for speed.
    while True:
        data = f.read(chunk_words * 4)
        if not data:
            return
        if len(data) % 4 != 0:
            raise ValueError(f"File length not multiple of 4 bytes (trailing {len(data) % 4} bytes).")

        n = len(data) // 4
        fmt = f"{fmt_prefix}{n}I"
        for w in struct.unpack(fmt, data):
            yield w


def decode_evt2_to_csv(
    in_path: str,
    out_path: str,
    endian: str = "little",
    x_max: int = DEFAULT_X_MAX,
    y_max: int = DEFAULT_Y_MAX,
    skip_until_time_high: bool = True,
    include_header: bool = True,
) -> None:
    """
    Decode EVT2 stream into CSV columns: x,y,polarity,timestamp_us
    - skip_until_time_high=True: ignore CD events until first EVT_TIME_HIGH arrives (recommended) :contentReference[oaicite:3]{index=3}
    """
    ts_high: Optional[int] = None
    total_words = 0
    cd_events_written = 0
    dropped_oob = 0
    dropped_before_th = 0

    with open(in_path, "rb") as f_in, open(out_path, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        if include_header:
            writer.writerow(["x", "y", "polarity", "timestamp_us"])

        for w in iter_words(f_in, endian=endian):
            total_words += 1
            w_type = (w >> 28) & 0xF

            if w_type == TYPE_TIME_HIGH:
                ts_high = w & 0x0FFFFFFF  # 28 bits
                continue

            if w_type == TYPE_CD_OFF or w_type == TYPE_CD_ON:
                if ts_high is None and skip_until_time_high:
                    dropped_before_th += 1
                    continue

                ts_low = (w >> 22) & 0x3F
                x = (w >> 11) & 0x7FF
                y = w & 0x7FF
                pol = 1 if (w_type == TYPE_CD_ON) else 0

                # Build 34-bit timestamp in microseconds. :contentReference[oaicite:4]{index=4}
                # ts_high corresponds to bits [33:6], ts_low to bits [5:0].
                timestamp_us = ((ts_high or 0) << 6) | ts_low

                # Optional bounds check for your sensor’s actual resolution.
                if x > x_max or y > y_max:
                    dropped_oob += 1
                    continue

                writer.writerow([x, y, pol, timestamp_us])
                cd_events_written += 1
                continue

            # Other types (EXT_TRIGGER=0xA, etc.) are ignored for this "x,y,pol,t" export. :contentReference[oaicite:5]{index=5}

    print(
        f"Done.\n"
        f"  Input:  {in_path}\n"
        f"  Output: {out_path}\n"
        f"  Endian: {endian}\n"
        f"  Words read: {total_words}\n"
        f"  CD events written: {cd_events_written}\n"
        f"  Dropped (out of bounds): {dropped_oob}\n"
        f"  Dropped (before first TIME_HIGH): {dropped_before_th}\n"
    )


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Decode a raw EVT2.0 .bin stream to CSV (x,y,polarity,timestamp_us)."
    )
    ap.add_argument("input_bin", help="Path to input .bin file containing raw EVT2 32-bit words (no ASCII header).")
    ap.add_argument(
        "-o",
        "--output",
        help="Path to output CSV/TXT file. Default: <input>.csv",
        default=None,
    )
    ap.add_argument(
        "--endian",
        choices=["little", "big"],
        default="little",
        help="Byte order of 32-bit words (default: little). EVT2 is commonly little-endian on GenX320/IMX636. :contentReference[oaicite:6]{index=6}",
    )
    ap.add_argument(
        "--x-max",
        type=int,
        default=DEFAULT_X_MAX,
        help="Max allowed x coordinate (inclusive). Set to your sensor width-1, e.g. 319 for 320-wide.",
    )
    ap.add_argument(
        "--y-max",
        type=int,
        default=DEFAULT_Y_MAX,
        help="Max allowed y coordinate (inclusive). Set to your sensor height-1, e.g. 271 for 272-tall.",
    )
    ap.add_argument(
        "--no-skip-until-time-high",
        action="store_true",
        help="Do NOT skip CD events before the first EVT_TIME_HIGH (not recommended). :contentReference[oaicite:7]{index=7}",
    )
    ap.add_argument(
        "--no-header",
        action="store_true",
        help="Do not write CSV header row.",
    )
    args = ap.parse_args()

    in_path = args.input_bin
    if not os.path.isfile(in_path):
        print(f"Error: input file not found: {in_path}", file=sys.stderr)
        return 2

    out_path = args.output
    if out_path is None:
        base, _ = os.path.splitext(in_path)
        out_path = base + ".csv"

    decode_evt2_to_csv(
        in_path=in_path,
        out_path=out_path,
        endian=args.endian,
        x_max=args.x_max,
        y_max=args.y_max,
        skip_until_time_high=(not args.no_skip_until_time_high),
        include_header=(not args.no_header),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
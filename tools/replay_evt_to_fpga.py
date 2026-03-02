#!/usr/bin/env python3
"""Replay or live-relay EVT2 events to FPGA for gesture classification validation."""

import argparse
import sys
import time
import struct
import threading
from collections import Counter, deque
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)


VALID_EVT2_TYPES = {0, 1, 8, 0xA, 0xE, 0xF}
EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

GESTURE_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}


def decode_evt2_word(word, x_shift, x_mask, y_shift, y_mask, swap_xy):
    event_type = (word >> 28) & 0xF

    if event_type == EVT_CD_OFF or event_type == EVT_CD_ON:
        x = (word >> x_shift) & x_mask
        y = (word >> y_shift) & y_mask
        if swap_xy:
            x, y = y, x
        pol = 1 if event_type == EVT_CD_ON else 0
        return event_type, x, y, pol

    return event_type, None, None, None


def encode_evt2_word_be(x, y, pol, ts_lsb=0):
    """Pack x, y, polarity into a 4-byte big-endian EVT2.0 CD word.

    voxel_bin_top expects raw EVT2.0 words, MSB first (big-endian).
    type=0x1 (CD_ON) if pol else 0x0 (CD_OFF); ts_lsb fills bits [27:22].
    """
    evt_type = 0x1 if pol else 0x0
    word = (evt_type << 28) | ((ts_lsb & 0x3F) << 22) | ((x & 0x7FF) << 11) | (y & 0x7FF)
    return word.to_bytes(4, "big")


def detect_alignment_offset(data, sample_words=20000):
    best = (0, -1.0)
    for off in range(4):
        n = min((len(data) - off) // 4, sample_words)
        if n <= 0:
            continue
        valid = 0
        for i in range(n):
            w = int.from_bytes(data[off + 4 * i: off + 4 * i + 4], "little")
            if ((w >> 28) & 0xF) in VALID_EVT2_TYPES:
                valid += 1
        ratio = valid / n
        if ratio > best[1]:
            best = (off, ratio)
    return best


class FPGAResponseReader:

    def __init__(self, port):
        self.port = port
        self.running = False
        self.thread = None
        self.gestures = deque()
        self.raw_lines = deque()
        self.gesture_count = Counter()
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def _worker(self):
        pending_gesture_byte = None
        ascii_buf = bytearray()
        while self.running:
            try:
                data = self.port.read(self.port.in_waiting or 1)
                if not data:
                    continue
                for b in data:
                    # Binary voxel_bin protocol: [0xA0|gesture, confidence]
                    if pending_gesture_byte is not None:
                        g = pending_gesture_byte & 0x03
                        gesture = ["UP", "DOWN", "LEFT", "RIGHT"][g]
                        with self.lock:
                            self.raw_lines.append(f"{gesture}(conf={(b >> 4) & 0xF})")
                            self.gestures.append(gesture)
                            self.gesture_count[gesture] += 1
                        pending_gesture_byte = None
                        continue

                    if (b & 0xF0) == 0xA0:
                        pending_gesture_byte = b
                        continue

                    # ASCII protocol (gradient_map): newline-terminated gesture names
                    if b in (0x0A, 0x0D):
                        if ascii_buf:
                            line = ascii_buf.decode("ascii", errors="replace").strip()
                            ascii_buf.clear()
                            with self.lock:
                                self.raw_lines.append(line)
                                if line.upper() in ("UP", "DOWN", "LEFT", "RIGHT"):
                                    self.gestures.append(line.upper())
                                    self.gesture_count[line.upper()] += 1
                    elif 32 <= b <= 126:
                        ascii_buf.append(b)
                        if len(ascii_buf) > 32:
                            ascii_buf.clear()
            except serial.SerialException:
                break
            except Exception:
                time.sleep(0.01)

    def get_gestures(self):
        with self.lock:
            out = list(self.gestures)
            self.gestures.clear()
            return out

    def get_raw_lines(self):
        with self.lock:
            out = list(self.raw_lines)
            self.raw_lines.clear()
            return out

    def get_summary(self):
        with self.lock:
            return dict(self.gesture_count)


def replay_file(args):
    filepath = Path(args.file)
    if not filepath.exists():
        print(f"ERROR: File not found: {filepath}")
        return 1

    data = filepath.read_bytes()
    print(f"Loaded {len(data)} bytes from {filepath}")
    off, ratio = detect_alignment_offset(data)
    if off > 0:
        print(f"Auto-alignment: offset={off}, valid ratio={ratio:.3f}")
        data = data[off:]
    data = data[: (len(data) // 4) * 4]
    total_words = len(data) // 4
    print(f"Total 32-bit words: {total_words}")
    cd_count = sum(1 for i in range(total_words)
                   if ((int.from_bytes(data[4*i:4*i+4], "little") >> 28) & 0xF) in (EVT_CD_OFF, EVT_CD_ON))
    print(f"CD events in file: {cd_count}")
    try:
        fpga = serial.Serial(args.fpga, args.fpga_baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open FPGA port {args.fpga}: {e}")
        return 1
    print(f"FPGA connected: {args.fpga} @ {args.fpga_baud} baud")
    reader = FPGAResponseReader(fpga)
    reader.start()
    inter_delay = 1.0 / args.rate if args.rate > 0 else 0.0
    print(f"Replaying {cd_count} CD events...")
    print(f"{'Time':>8s}  {'Sent':>7s}  {'Gesture':>10s}  {'Totals'}")
    print(f"{'-'*70}")

    sent = 0
    skipped = 0
    start_time = time.time()
    last_print = start_time

    try:
        for i in range(total_words):
            w = int.from_bytes(data[4 * i: 4 * i + 4], "little")
            evt_type = (w >> 28) & 0xF

            if evt_type not in VALID_EVT2_TYPES:
                continue

            if evt_type in (EVT_CD_OFF, EVT_CD_ON):
                _, x, y, pol = decode_evt2_word(
                    w, args.x_shift, args.x_mask, args.y_shift, args.y_mask, args.swap_xy
                )
                if x >= 320 or y >= 320:
                    skipped += 1
                    continue
                pkt = encode_evt2_word_be(x, y, pol)
            else:
                # Pass non-CD valid types (e.g. TIME_HIGH) through unchanged, LE→BE.
                # The FPGA decoder needs TIME_HIGH events for timestamp reconstruction.
                pkt = w.to_bytes(4, "big")

            fpga.write(pkt)
            sent += 1

            if inter_delay > 0 and evt_type in (EVT_CD_OFF, EVT_CD_ON):
                time.sleep(inter_delay)

            # Periodic status output
            now = time.time()
            if now - last_print >= 1.0:
                elapsed = now - start_time
                gestures = reader.get_gestures()
                gesture_str = ", ".join(gestures) if gestures else "-"
                summary = reader.get_summary()
                totals = " | ".join(f"{k}:{v}" for k, v in sorted(summary.items()))
                print(f"{elapsed:7.1f}s  {sent:7d}  {gesture_str:>10s}  {totals}")
                last_print = now

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    time.sleep(1.0)
    for g in reader.get_gestures():
        print(f"  Gesture: {g}")
    reader.stop()
    fpga.close()
    elapsed = time.time() - start_time
    summary = reader.get_summary()
    print(f"\nREPLAY COMPLETE: {sent} events sent, {skipped} skipped, {elapsed:.1f}s")
    total_gestures = sum(summary.values())
    if total_gestures == 0:
        print("  (no gestures detected)")
    else:
        for gesture in ["UP", "DOWN", "LEFT", "RIGHT"]:
            count = summary.get(gesture, 0)
            pct = 100 * count / total_gestures if total_gestures > 0 else 0
            print(f"  {gesture:>5s}: {count:4d} ({pct:5.1f}%) {'#' * int(pct / 2)}")
        print(f"  TOTAL: {total_gestures}")
    return 0


def live_relay(args):
    print(f"Opening DVS port: {args.dvs} @ {args.dvs_baud}")
    try:
        dvs = serial.Serial(args.dvs, args.dvs_baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open DVS port {args.dvs}: {e}")
        return 1

    print(f"Opening FPGA port: {args.fpga} @ {args.fpga_baud}")
    try:
        fpga = serial.Serial(args.fpga, args.fpga_baud, timeout=0.1)
    except serial.SerialException as e:
        dvs.close()
        print(f"ERROR: Cannot open FPGA port {args.fpga}: {e}")
        return 1

    save_file = None
    if args.save:
        save_file = open(args.save, "wb")
        print(f"Saving raw capture to: {args.save}")
    reader = FPGAResponseReader(fpga)
    reader.start()
    print(f"Probing stream alignment...")
    probe = dvs.read(4096)
    if save_file:
        save_file.write(probe)
    if len(probe) >= 16:
        off, ratio = detect_alignment_offset(probe)
        print(f"  Alignment offset: {off} (valid ratio: {ratio:.3f})")
        probe = probe[off:]
    else:
        print(f"  Not enough data for alignment ({len(probe)} bytes)")
    buf = bytearray(probe)
    sent = 0
    skipped = 0
    words = 0
    start_time = time.time()
    last_print = start_time
    print(f"Live relay started. Press Ctrl+C to stop.")
    print(f"{'Time':>8s}  {'Words':>8s}  {'Sent':>7s}  {'Gesture':>10s}  {'Totals'}")
    print(f"{'-'*70}")

    try:
        while True:
            # Read from DVS
            data = dvs.read(dvs.in_waiting or 1)
            if data:
                if save_file:
                    save_file.write(data)
                buf.extend(data)

            # Process complete 32-bit words
            n = (len(buf) // 4) * 4
            for i in range(0, n, 4):
                w = int.from_bytes(buf[i:i + 4], "little")
                words += 1
                evt_type = (w >> 28) & 0xF

                if evt_type not in VALID_EVT2_TYPES:
                    continue

                if evt_type in (EVT_CD_OFF, EVT_CD_ON):
                    _, x, y, pol = decode_evt2_word(
                        w, args.x_shift, args.x_mask,
                        args.y_shift, args.y_mask, args.swap_xy
                    )
                    if x >= 320 or y >= 320:
                        skipped += 1
                        continue
                    pkt = encode_evt2_word_be(x, y, pol)
                else:
                    pkt = w.to_bytes(4, "big")

                fpga.write(pkt)
                sent += 1

            buf = buf[n:]
            now = time.time()
            if now - last_print >= 1.0:
                elapsed = now - start_time
                gestures = reader.get_gestures()
                gesture_str = ", ".join(gestures) if gestures else "-"
                summary = reader.get_summary()
                totals = " | ".join(f"{k}:{v}" for k, v in sorted(summary.items()))
                print(
                    f"{elapsed:7.1f}s  {words:8d}  {sent:7d}"
                    f"  {gesture_str:>10s}  {totals}"
                )
                last_print = now

            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                print(f"\nDuration limit ({args.duration}s) reached.")
                break
    except KeyboardInterrupt:
        print("\nStopped by user")
    time.sleep(0.5)
    for g in reader.get_gestures():
        print(f"  Final gesture: {g}")
    reader.stop()
    dvs.close()
    fpga.close()
    if save_file:
        save_file.close()
    elapsed = time.time() - start_time
    summary = reader.get_summary()
    print(f"\nRELAY COMPLETE: {sent} events sent, {skipped} skipped, {elapsed:.1f}s")
    total_gestures = sum(summary.values())
    if total_gestures == 0:
        print("  (no gestures detected)")
    else:
        for gesture in ["UP", "DOWN", "LEFT", "RIGHT"]:
            count = summary.get(gesture, 0)
            pct = 100 * count / total_gestures if total_gestures > 0 else 0
            print(f"  {gesture:>5s}: {count:4d} ({pct:5.1f}%) {'#' * int(pct / 2)}")
        print(f"  TOTAL: {total_gestures}")
    return 0


def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        extra = f" [USB {p.vid:04X}:{p.pid:04X}]" if p.vid is not None else ""
        print(f"  {p.device:12s} {p.description}{extra}")


def main():
    parser = argparse.ArgumentParser(description="Replay or relay EVT2 events to FPGA")
    parser.add_argument("--file", "-f", type=str)
    parser.add_argument("--dvs", type=str)
    parser.add_argument("--fpga", type=str)
    parser.add_argument("--fpga-baud", type=int, default=115200)
    parser.add_argument("--dvs-baud", type=int, default=115200)
    parser.add_argument("--rate", type=int, default=0)
    parser.add_argument("--save", type=str)
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--x-shift", type=int, default=11)
    parser.add_argument("--x-mask", type=lambda v: int(v, 0), default=0x7FF)
    parser.add_argument("--y-shift", type=int, default=0)
    parser.add_argument("--y-mask", type=lambda v: int(v, 0), default=0x7FF)
    parser.add_argument("--swap-xy", action="store_true", default=False)
    parser.add_argument("--no-swap-xy", dest="swap_xy", action="store_false")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()
    if args.list_ports:
        list_ports()
        return 0
    if not args.fpga:
        parser.print_help()
        print("\nERROR: --fpga is required")
        return 1
    if args.file and args.dvs:
        print("ERROR: Cannot use both --file and --dvs.")
        return 1
    if not args.file and not args.dvs:
        parser.print_help()
        print("\nERROR: Specify --file (replay) or --dvs (live relay)")
        return 1
    if args.file:
        return replay_file(args)
    else:
        return live_relay(args)


if __name__ == "__main__":
    sys.exit(main())

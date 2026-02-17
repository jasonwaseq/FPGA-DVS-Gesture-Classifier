#!/usr/bin/env python3
"""
Replay captured EVT2 stream to FPGA for hardware gesture classification validation.

Modes:
  1. File replay:   Read a captured .bin file and send events to FPGA via UART
  2. Live relay:    Read live EVT2 stream from STM32/GenX320 and forward to FPGA
  3. Capture+relay: Capture raw stream to file while simultaneously relaying to FPGA

The FPGA (gesture_uart_top) receives 5-byte event packets over UART and outputs
gesture classifications as ASCII strings ("UP\\r\\n", "DOWN\\r\\n", etc.).

Usage:
    # Replay a previously captured file
    python replay_evt_to_fpga.py --file aligned.bin --fpga COM3

    # Live relay from STM32 to FPGA
    python replay_evt_to_fpga.py --dvs COM5 --fpga COM3

    # Capture + relay simultaneously
    python replay_evt_to_fpga.py --dvs COM5 --fpga COM3 --save capture.bin

    # List available serial ports
    python replay_evt_to_fpga.py --list-ports

    # Replay with custom EVT2 bit layout
    python replay_evt_to_fpga.py --file aligned.bin --fpga COM3 --x-shift 11 --y-shift 0
"""

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


# =============================================================================
# EVT2 Constants
# =============================================================================

VALID_EVT2_TYPES = {0, 1, 8, 0xA, 0xE, 0xF}
EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

GESTURE_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}


# =============================================================================
# EVT2 Decoding
# =============================================================================

def decode_evt2_word(word, x_shift, x_mask, y_shift, y_mask, swap_xy):
    """Decode a raw 32-bit EVT2 word into (event_type, x, y, polarity).

    Returns (event_type, x, y, pol) or (event_type, None, None, None) for
    non-CD events.
    """
    event_type = (word >> 28) & 0xF

    if event_type == EVT_CD_OFF or event_type == EVT_CD_ON:
        x = (word >> x_shift) & x_mask
        y = (word >> y_shift) & y_mask
        if swap_xy:
            x, y = y, x
        pol = 1 if event_type == EVT_CD_ON else 0
        return event_type, x, y, pol

    return event_type, None, None, None


def encode_5byte_packet(x, y, pol):
    """Encode a DVS event into the FPGA's 5-byte UART protocol.

    Packet: [X_HI, X_LO, Y_HI, Y_LO, POL]
    """
    return bytes([
        (x >> 8) & 0x01,  # X[8]
        x & 0xFF,          # X[7:0]
        (y >> 8) & 0x01,  # Y[8]
        y & 0xFF,          # Y[7:0]
        pol & 0x01,        # Polarity
    ])


def detect_alignment_offset(data, sample_words=20000):
    """Find the byte offset that maximizes the ratio of valid EVT2 type fields."""
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


# =============================================================================
# FPGA Response Reader (background thread)
# =============================================================================

class FPGAResponseReader:
    """Reads gesture ASCII responses from the FPGA's UART TX in a background thread."""

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
        buf = b""
        while self.running:
            try:
                data = self.port.read(self.port.in_waiting or 1)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip(b"\r").decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    with self.lock:
                        self.raw_lines.append(line)
                        gesture = line.upper()
                        if gesture in ("UP", "DOWN", "LEFT", "RIGHT"):
                            self.gestures.append(gesture)
                            self.gesture_count[gesture] += 1
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


# =============================================================================
# File Replay Mode
# =============================================================================

def replay_file(args):
    """Replay a captured .bin file to the FPGA over UART."""
    filepath = Path(args.file)
    if not filepath.exists():
        print(f"ERROR: File not found: {filepath}")
        return 1

    data = filepath.read_bytes()
    print(f"Loaded {len(data)} bytes from {filepath}")

    # Auto-align if needed
    off, ratio = detect_alignment_offset(data)
    if off > 0:
        print(f"Auto-alignment: offset={off}, valid ratio={ratio:.3f}")
        data = data[off:]
    data = data[: (len(data) // 4) * 4]
    total_words = len(data) // 4
    print(f"Total 32-bit words: {total_words}")

    # Count CD events to estimate duration
    cd_count = 0
    for i in range(total_words):
        w = int.from_bytes(data[4 * i: 4 * i + 4], "little")
        t = (w >> 28) & 0xF
        if t == EVT_CD_OFF or t == EVT_CD_ON:
            cd_count += 1
    print(f"CD events in file: {cd_count}")

    # Open FPGA port
    print(f"\nConnecting to FPGA on {args.fpga}...")
    try:
        fpga = serial.Serial(args.fpga, args.fpga_baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open FPGA port {args.fpga}: {e}")
        return 1
    print(f"FPGA connected: {args.fpga} @ {args.fpga_baud} baud")

    # Start response reader
    reader = FPGAResponseReader(fpga)
    reader.start()

    # Calculate inter-event delay for target event rate
    if args.rate > 0:
        inter_delay = 1.0 / args.rate
    else:
        inter_delay = 0.0  # Send as fast as UART allows

    # Replay
    print(f"\nReplaying {cd_count} CD events to FPGA...")
    if args.rate > 0:
        print(f"Target rate: {args.rate} events/sec ({inter_delay * 1e6:.0f} us/event)")
    else:
        print(f"Rate: maximum (UART limited, ~2300 evt/s at 115200 baud)")
    print(f"{'='*70}")
    print(f"{'Time':>8s}  {'Sent':>7s}  {'Gesture':>10s}  {'Totals'}")
    print(f"{'-'*70}")

    sent = 0
    skipped = 0
    start_time = time.time()
    last_print = start_time

    try:
        for i in range(total_words):
            w = int.from_bytes(data[4 * i: 4 * i + 4], "little")
            evt_type, x, y, pol = decode_evt2_word(
                w, args.x_shift, args.x_mask, args.y_shift, args.y_mask, args.swap_xy
            )

            if x is None:
                continue  # Not a CD event

            if x >= 320 or y >= 320:
                skipped += 1
                continue

            pkt = encode_5byte_packet(x, y, pol)
            fpga.write(pkt)
            sent += 1

            if inter_delay > 0:
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
        print("\n\nInterrupted by user")

    # Wait for final classifications
    print(f"\nWaiting for final FPGA responses...")
    time.sleep(1.0)

    # Print remaining gestures
    gestures = reader.get_gestures()
    for g in gestures:
        print(f"  Gesture: {g}")

    reader.stop()
    fpga.close()

    # Summary
    elapsed = time.time() - start_time
    summary = reader.get_summary()

    print(f"\n{'='*70}")
    print(f"REPLAY COMPLETE")
    print(f"{'='*70}")
    print(f"  File: {filepath}")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Events sent: {sent}")
    print(f"  Events skipped (out of range): {skipped}")
    print(f"  Effective rate: {sent / elapsed:.0f} events/sec" if elapsed > 0 else "")
    print(f"\n  Gesture detections:")
    total_gestures = sum(summary.values())
    if total_gestures == 0:
        print(f"    (none detected)")
    else:
        for gesture in ["UP", "DOWN", "LEFT", "RIGHT"]:
            count = summary.get(gesture, 0)
            pct = 100 * count / total_gestures if total_gestures > 0 else 0
            bar = "#" * int(pct / 2)
            print(f"    {gesture:>5s}: {count:4d} ({pct:5.1f}%) {bar}")
        print(f"    {'TOTAL':>5s}: {total_gestures:4d}")
    print(f"{'='*70}")

    return 0


# =============================================================================
# Live Relay Mode
# =============================================================================

def live_relay(args):
    """Live relay: read EVT2 from STM32, forward 5-byte packets to FPGA."""
    print(f"\nOpening DVS port: {args.dvs} @ {args.dvs_baud}")
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

    # Optional file save
    save_file = None
    if args.save:
        save_file = open(args.save, "wb")
        print(f"Saving raw capture to: {args.save}")

    # Start FPGA response reader
    reader = FPGAResponseReader(fpga)
    reader.start()

    # Alignment probe: read initial chunk to find byte alignment
    print(f"\nProbing stream alignment...")
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

    print(f"\nLive relay started. Press Ctrl+C to stop.")
    print(f"{'='*70}")
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

                evt_type, x, y, pol = decode_evt2_word(
                    w, args.x_shift, args.x_mask,
                    args.y_shift, args.y_mask, args.swap_xy
                )

                if x is None:
                    continue

                if x >= 320 or y >= 320:
                    skipped += 1
                    continue

                pkt = encode_5byte_packet(x, y, pol)
                fpga.write(pkt)
                sent += 1

            buf = buf[n:]

            # Status output
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

            # Duration limit
            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                print(f"\nDuration limit ({args.duration}s) reached.")
                break

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    # Cleanup
    time.sleep(0.5)
    gestures = reader.get_gestures()
    for g in gestures:
        print(f"  Final gesture: {g}")

    reader.stop()
    dvs.close()
    fpga.close()
    if save_file:
        save_file.close()

    elapsed = time.time() - start_time
    summary = reader.get_summary()

    print(f"\n{'='*70}")
    print(f"RELAY COMPLETE")
    print(f"{'='*70}")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  EVT2 words processed: {words}")
    print(f"  Events sent to FPGA: {sent}")
    print(f"  Events skipped (out of range): {skipped}")
    print(f"  Effective rate: {sent / elapsed:.0f} events/sec" if elapsed > 0 else "")
    if args.save:
        print(f"  Raw capture saved to: {args.save}")
    print(f"\n  Gesture detections:")
    total_gestures = sum(summary.values())
    if total_gestures == 0:
        print(f"    (none detected)")
    else:
        for gesture in ["UP", "DOWN", "LEFT", "RIGHT"]:
            count = summary.get(gesture, 0)
            pct = 100 * count / total_gestures if total_gestures > 0 else 0
            bar = "#" * int(pct / 2)
            print(f"    {gesture:>5s}: {count:4d} ({pct:5.1f}%) {bar}")
        print(f"    {'TOTAL':>5s}: {total_gestures:4d}")
    print(f"{'='*70}")

    return 0


# =============================================================================
# Utility
# =============================================================================

def list_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for p in ports:
        extra = ""
        if p.vid is not None:
            extra = f" [USB {p.vid:04X}:{p.pid:04X}]"
        print(f"  {p.device:12s} {p.description}{extra}")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Replay or relay EVT2 events to FPGA for gesture classification",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EVT2 bit layout presets:
  Standard EVT2 (default):  --x-shift 11 --y-shift 0 --no-swap-xy
  GenX320 relay format:     --x-shift 17 --y-shift 6 --swap-xy

Examples:
  %(prog)s --file aligned.bin --fpga COM3
  %(prog)s --dvs COM5 --fpga COM3
  %(prog)s --dvs COM5 --fpga COM3 --save capture.bin --duration 30
  %(prog)s --list-ports
        """
    )

    # Mode selection
    parser.add_argument("--file", "-f", type=str,
                        help="Replay a captured .bin file (file replay mode)")
    parser.add_argument("--dvs", type=str,
                        help="DVS camera serial port for live relay (e.g. COM5, /dev/ttyACM0)")

    # FPGA port
    parser.add_argument("--fpga", type=str,
                        help="FPGA serial port (e.g. COM3, /dev/ttyUSB1)")

    # Common options
    parser.add_argument("--fpga-baud", type=int, default=115200,
                        help="FPGA UART baud rate (default: 115200)")
    parser.add_argument("--dvs-baud", type=int, default=115200,
                        help="DVS port baud rate (default: 115200)")

    # File replay options
    parser.add_argument("--rate", type=int, default=0,
                        help="Target event rate in events/sec (0 = max UART speed, default: 0)")

    # Live relay options
    parser.add_argument("--save", type=str,
                        help="Save raw capture to file while relaying")
    parser.add_argument("--duration", type=float, default=0,
                        help="Relay duration in seconds (0 = until Ctrl+C, default: 0)")

    # EVT2 bit layout (configurable for different sensor firmware)
    parser.add_argument("--x-shift", type=int, default=11,
                        help="X field bit shift (default: 11, standard EVT2)")
    parser.add_argument("--x-mask", type=lambda v: int(v, 0), default=0x7FF,
                        help="X field mask (default: 0x7FF)")
    parser.add_argument("--y-shift", type=int, default=0,
                        help="Y field bit shift (default: 0, standard EVT2)")
    parser.add_argument("--y-mask", type=lambda v: int(v, 0), default=0x7FF,
                        help="Y field mask (default: 0x7FF)")
    parser.add_argument("--swap-xy", action="store_true", default=False,
                        help="Swap X/Y after extraction")
    parser.add_argument("--no-swap-xy", dest="swap_xy", action="store_false")

    # Utility
    parser.add_argument("--list-ports", action="store_true",
                        help="List available serial ports and exit")

    args = parser.parse_args()

    # Utility mode
    if args.list_ports:
        list_ports()
        return 0

    # Validate arguments
    if not args.fpga:
        if not args.file and not args.dvs:
            parser.print_help()
            print("\nERROR: Specify --file (replay) or --dvs (live relay), plus --fpga")
            return 1
        parser.print_help()
        print("\nERROR: --fpga is required")
        return 1

    if args.file and args.dvs:
        print("ERROR: Cannot use both --file and --dvs. Choose one mode.")
        return 1

    if not args.file and not args.dvs:
        parser.print_help()
        print("\nERROR: Specify --file (replay) or --dvs (live relay)")
        return 1

    # Banner
    print(f"\n{'='*70}")
    print(f"  EVT2 -> FPGA Gesture Validator")
    print(f"{'='*70}")
    if args.file:
        print(f"  Mode:     File replay")
        print(f"  File:     {args.file}")
    else:
        print(f"  Mode:     Live relay")
        print(f"  DVS port: {args.dvs}")
    print(f"  FPGA:     {args.fpga} @ {args.fpga_baud} baud")
    print(f"  EVT2 layout: x_shift={args.x_shift}, y_shift={args.y_shift}, "
          f"swap_xy={args.swap_xy}")
    print(f"{'='*70}")

    # Dispatch
    if args.file:
        return replay_file(args)
    else:
        return live_relay(args)


if __name__ == "__main__":
    sys.exit(main())

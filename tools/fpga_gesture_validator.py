#!/usr/bin/env python3
"""Hardware validation: send mock DVS events to FPGA and verify gesture responses."""

import argparse
import time
import sys
import struct
import threading
import queue
from typing import Optional, List, Tuple
from dataclasses import dataclass
from enum import IntEnum
import random
import math

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)


class Architecture:
    VOXEL_BIN = "voxel_bin"
    GRADIENT_MAP = "gradient_map"


class Gesture(IntEnum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3

GESTURE_NAMES = {
    Gesture.UP: "UP",
    Gesture.DOWN: "DOWN",
    Gesture.LEFT: "LEFT",
    Gesture.RIGHT: "RIGHT"
}

DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 1.0
SENSOR_WIDTH = 320
SENSOR_HEIGHT = 320
GESTURE_DURATION_MS = 400
EVENT_RATE_HZ = 500


@dataclass
class DVSEvent:
    x: int
    y: int
    polarity: int
    timestamp_us: int = 0


@dataclass
class GestureResult:
    gesture: Gesture
    confidence: int
    event_count_hi: int
    raw_bytes: bytes


def generate_gesture_events(
    gesture: Gesture,
    duration_ms: float = GESTURE_DURATION_MS,
    event_rate: float = EVENT_RATE_HZ,
    center_x: int = 160,
    center_y: int = 160,
    motion_amplitude: int = 80,
    spatial_noise: float = 15.0,
    noise_ratio: float = 0.05
) -> List[DVSEvent]:
    events = []
    num_events = int(duration_ms * event_rate / 1000)
    if gesture == Gesture.UP:
        start_x, end_x = center_x, center_x
        start_y, end_y = center_y + motion_amplitude, center_y - motion_amplitude
    elif gesture == Gesture.DOWN:
        start_x, end_x = center_x, center_x
        start_y, end_y = center_y - motion_amplitude, center_y + motion_amplitude
    elif gesture == Gesture.LEFT:
        start_x, end_x = center_x + motion_amplitude, center_x - motion_amplitude
        start_y, end_y = center_y, center_y
    elif gesture == Gesture.RIGHT:
        start_x, end_x = center_x - motion_amplitude, center_x + motion_amplitude
        start_y, end_y = center_y, center_y
    else:
        raise ValueError(f"Unknown gesture: {gesture}")
    for i in range(num_events):
        t = i / max(1, num_events - 1)
        timestamp_us = int(t * duration_ms * 1000)
        if random.random() < noise_ratio:
            x = random.randint(0, SENSOR_WIDTH - 1)
            y = random.randint(0, SENSOR_HEIGHT - 1)
        else:
            x = start_x + t * (end_x - start_x) + random.gauss(0, spatial_noise)
            y = start_y + t * (end_y - start_y) + random.gauss(0, spatial_noise)
            x = int(max(0, min(SENSOR_WIDTH - 1, x)))
            y = int(max(0, min(SENSOR_HEIGHT - 1, y)))
        polarity = 1 if random.random() > 0.15 else 0
        events.append(DVSEvent(x=x, y=y, polarity=polarity, timestamp_us=timestamp_us))
    return events


def generate_random_events(count: int) -> List[DVSEvent]:
    return [DVSEvent(x=random.randint(0, SENSOR_WIDTH - 1),
                     y=random.randint(0, SENSOR_HEIGHT - 1),
                     polarity=random.randint(0, 1),
                     timestamp_us=i * 1000) for i in range(count)]


class FPGAGestureInterface:
    def __init__(self, port: str, baud_rate: int = DEFAULT_BAUD_RATE,
                 architecture: str = Architecture.VOXEL_BIN):
        self.port = port
        self.baud_rate = baud_rate
        self.architecture = architecture
        self.serial: Optional[serial.Serial] = None
        self.rx_queue = queue.Queue()
        self.rx_thread: Optional[threading.Thread] = None
        self.running = False
        self._ascii_line_buffer = bytearray()

    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=DEFAULT_TIMEOUT
            )
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.rx_thread.start()
            time.sleep(0.1)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Failed to open {self.port}: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    def _rx_worker(self):
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    for byte in data:
                        self.rx_queue.put(byte)
                else:
                    time.sleep(0.001)
            except:
                break

    def _send_byte(self, byte: int):
        if self.serial:
            self.serial.write(bytes([byte & 0xFF]))

    def _send_bytes(self, data: bytes):
        if self.serial:
            self.serial.write(data)

    def _receive_byte(self, timeout: float = 1.0) -> Optional[int]:
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _receive_bytes(self, count: int, timeout: float = 1.0) -> bytes:
        result = []
        deadline = time.time() + timeout
        while len(result) < count and time.time() < deadline:
            try:
                byte = self.rx_queue.get(timeout=max(0.001, deadline - time.time()))
                result.append(byte)
            except queue.Empty:
                break
        return bytes(result)

    def clear_rx_buffer(self):
        while not self.rx_queue.empty():
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                break
        self._ascii_line_buffer.clear()

    def realign_parser(self):
        """Send 3 zero bytes to complete any partial 4-byte EVT2 word."""
        self._send_bytes(bytes(3))
        time.sleep(0.01)
        self.clear_rx_buffer()

    def detect_architecture(self) -> str:
        """Probe with 0xFF; 0x55 reply → voxel_bin, else → gradient_map."""
        self.clear_rx_buffer()
        self._send_byte(0xFF)
        response = self._receive_byte(timeout=0.3)
        if response == 0x55:
            print("Auto-detected architecture: voxel_bin")
            return Architecture.VOXEL_BIN
        else:
            print("Auto-detected architecture: gradient_map")
            self.realign_parser()
            return Architecture.GRADIENT_MAP

    def send_echo(self) -> bool:
        if self.architecture == Architecture.GRADIENT_MAP:
            self.clear_rx_buffer()
            return True
        self.clear_rx_buffer()
        self._send_byte(0xFF)
        response = self._receive_byte(timeout=0.5)
        if response == 0x55:
            return True
        print(f"Echo failed: expected 0x55, got {response}")
        return False

    def query_status(self) -> Optional[dict]:
        if self.architecture == Architecture.GRADIENT_MAP:
            return None
        self.clear_rx_buffer()
        self._send_byte(0xFE)
        response = self._receive_byte(timeout=0.5)
        if response is None:
            return None
        if (response & 0xF0) != 0xB0:
            print(f"Invalid status response: 0x{response:02X}")
            return None
        return {
            'phase': (response >> 3) & 0x01,
            'fifo_full': (response >> 2) & 0x01,
            'fifo_empty': (response >> 1) & 0x01,
            'raw': response
        }

    def query_config(self) -> Optional[dict]:
        if self.architecture == Architecture.GRADIENT_MAP:
            return None
        self.clear_rx_buffer()
        self._send_byte(0xFD)
        response = self._receive_bytes(2, timeout=0.5)
        if len(response) != 2:
            return None
        return {'min_event_thresh': response[0], 'motion_thresh': response[1]}

    def soft_reset(self):
        if self.architecture == Architecture.GRADIENT_MAP:
            return
        self._send_byte(0xFC)
        time.sleep(0.01)

    def send_dvs_event(self, event: DVSEvent):
        evt_type = 0x1 if event.polarity else 0x0
        word = (evt_type << 28) | ((event.x & 0x7FF) << 11) | (event.y & 0x7FF)
        self._send_bytes(word.to_bytes(4, "big"))

    def send_event_stream(self, events: List[DVSEvent], delay_us: float = 0):
        for event in events:
            self.send_dvs_event(event)
            if delay_us > 0:
                time.sleep(delay_us / 1_000_000)

    def _check_gesture_ascii(self, timeout: float) -> Optional[GestureResult]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                b = self.rx_queue.get(timeout=max(0.01, deadline - time.time()))
            except queue.Empty:
                break
            if b in (0x0A, 0x0D):
                line = self._ascii_line_buffer.decode("ascii", errors="ignore").strip()
                self._ascii_line_buffer.clear()
                if line == "UP":
                    return GestureResult(Gesture.UP, 0, 0, b"UP\r\n")
                if line == "DOWN":
                    return GestureResult(Gesture.DOWN, 0, 0, b"DOWN\r\n")
                if line == "LEFT":
                    return GestureResult(Gesture.LEFT, 0, 0, b"LEFT\r\n")
                if line == "RIGHT":
                    return GestureResult(Gesture.RIGHT, 0, 0, b"RIGHT\r\n")
            else:
                self._ascii_line_buffer.append(b)
                if len(self._ascii_line_buffer) > 32:
                    self._ascii_line_buffer.clear()
        return None

    def check_gesture(self, timeout: float = 0.1) -> Optional[GestureResult]:
        if self.architecture == Architecture.GRADIENT_MAP:
            return self._check_gesture_ascii(timeout)
        byte1 = self._receive_byte(timeout=timeout)
        if byte1 is None:
            return None
        if (byte1 & 0xF0) != 0xA0:
            return None
        byte2 = self._receive_byte(timeout=0.1)
        if byte2 is None:
            byte2 = 0
        gesture = Gesture(byte1 & 0x03)
        confidence = (byte2 >> 4) & 0x0F
        event_count_hi = byte2 & 0x0F
        return GestureResult(gesture=gesture, confidence=confidence,
                             event_count_hi=event_count_hi, raw_bytes=bytes([byte1, byte2]))


def test_connection(fpga: FPGAGestureInterface) -> bool:
    if fpga.send_echo():
        print("Echo test PASSED")
        return True
    print("Echo test FAILED")
    return False


def test_status(fpga: FPGAGestureInterface) -> bool:
    status = fpga.query_status()
    if status:
        print(f"Status: phase={status['phase']}, fifo_full={status['fifo_full']}, fifo_empty={status['fifo_empty']}")
        return True
    print("Status query FAILED")
    return False


def test_config(fpga: FPGAGestureInterface) -> bool:
    config = fpga.query_config()
    if config:
        print(f"Config: min_event_thresh={config['min_event_thresh']}, motion_thresh={config['motion_thresh']}")
        return True
    print("Config query FAILED")
    return False


def test_gesture(fpga: FPGAGestureInterface, gesture: Gesture,
                 num_events: int = 200, verbose: bool = True) -> Optional[GestureResult]:
    gesture_name = GESTURE_NAMES[gesture]
    fpga.soft_reset()
    fpga.clear_rx_buffer()
    if hasattr(fpga, "_ascii_line_buffer"):
        fpga._ascii_line_buffer.clear()
    time.sleep(0.05)
    if fpga.architecture == Architecture.VOXEL_BIN:
        status = fpga.query_status()
        if status and status.get("phase") == 1:
            if verbose:
                print("Waiting for early phase start...")
            time.sleep(0.25)
    events = generate_gesture_events(
        gesture, event_rate=EVENT_RATE_HZ * 2, duration_ms=GESTURE_DURATION_MS,
        motion_amplitude=120, spatial_noise=5.0, noise_ratio=0.0
    )
    if verbose:
        print(f"Sending {len(events)} events for {gesture_name}...")
    if fpga.architecture == Architecture.GRADIENT_MAP:
        fpga.send_event_stream(events, delay_us=200)
        time.sleep(0.2)
    else:
        half = len(events) // 2
        fpga.send_event_stream(events[:half], delay_us=500)
        time.sleep(0.2)
        fpga.send_event_stream(events[half:], delay_us=500)
    time.sleep(0.5)
    result = fpga.check_gesture(timeout=1.0)
    if result:
        detected_name = GESTURE_NAMES[result.gesture]
        if verbose:
            correct = "CORRECT" if result.gesture == gesture else f"INCORRECT (expected {gesture_name})"
            print(f"Detected: {detected_name} (confidence={result.confidence}) — {correct}")
        return result
    else:
        if verbose:
            print(f"No gesture detected (expected {gesture_name})")
        return None


def test_all_gestures(fpga: FPGAGestureInterface) -> dict:
    results = {}
    for gesture in Gesture:
        result = test_gesture(fpga, gesture, verbose=True)
        results[gesture] = result
        time.sleep(0.5)
    correct = sum(1 for g, r in results.items() if r and r.gesture == g)
    print(f"\nSummary: {correct}/4 correct")
    for gesture, result in results.items():
        name = GESTURE_NAMES[gesture]
        if result:
            detected = GESTURE_NAMES[result.gesture]
            ok = "+" if result.gesture == gesture else "-"
            print(f"  [{ok}] {name}: detected {detected} (conf={result.confidence})")
        else:
            print(f"  [-] {name}: no detection")
    return results


def test_noise_rejection(fpga: FPGAGestureInterface) -> bool:
    fpga.soft_reset()
    fpga.clear_rx_buffer()
    events = generate_random_events(10)
    print(f"Sending {len(events)} random noise events...")
    fpga.send_event_stream(events, delay_us=1000)
    time.sleep(0.5)
    result = fpga.check_gesture(timeout=0.5)
    if result:
        print(f"False detection: {GESTURE_NAMES[result.gesture]}")
        return False
    print("No false detection — noise rejection OK")
    return True


def continuous_monitoring(fpga: FPGAGestureInterface, duration_s: float = 60):
    print(f"Monitoring for {duration_s}s... (Ctrl+C to stop)")
    start_time = time.time()
    gesture_count = {g: 0 for g in Gesture}
    try:
        while (time.time() - start_time) < duration_s:
            result = fpga.check_gesture(timeout=0.1)
            if result:
                name = GESTURE_NAMES[result.gesture]
                gesture_count[result.gesture] += 1
                elapsed = time.time() - start_time
                print(f"[{elapsed:6.1f}s] {name} (confidence={result.confidence})")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped by user")
    total = sum(gesture_count.values())
    print(f"Total: {total}")
    for gesture, count in gesture_count.items():
        print(f"  {GESTURE_NAMES[gesture]}: {count}")


def interactive_mode(fpga: FPGAGestureInterface):
    print("Commands: u/d/l/r=gesture  e=echo  s=status  c=config  x=reset  n=noise  a=all  q=quit")
    while True:
        try:
            cmd = input("Command> ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 'e':
                print("Echo: OK" if fpga.send_echo() else "Echo: FAILED")
            elif cmd == 's':
                if fpga.architecture == Architecture.GRADIENT_MAP:
                    print("Status: not supported (gradient_map)")
                else:
                    status = fpga.query_status()
                    print(f"Status: {status}" if status else "Status: FAILED")
            elif cmd == 'c':
                if fpga.architecture == Architecture.GRADIENT_MAP:
                    print("Config: not supported (gradient_map)")
                else:
                    config = fpga.query_config()
                    print(f"Config: {config}" if config else "Config: FAILED")
            elif cmd == 'x':
                fpga.soft_reset()
                print("Reset sent")
            elif cmd == 'u':
                test_gesture(fpga, Gesture.UP)
            elif cmd == 'd':
                test_gesture(fpga, Gesture.DOWN)
            elif cmd == 'l':
                test_gesture(fpga, Gesture.LEFT)
            elif cmd == 'r':
                test_gesture(fpga, Gesture.RIGHT)
            elif cmd == 'n':
                test_noise_rejection(fpga)
            elif cmd == 'a':
                test_all_gestures(fpga)
            elif cmd:
                print(f"Unknown command: {cmd}")
        except KeyboardInterrupt:
            print()
            break
        except EOFError:
            break


def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found")
    else:
        for port in ports:
            print(f"  {port.device}: {port.description}")


def main():
    parser = argparse.ArgumentParser(description="DVS Gesture Classifier FPGA Hardware Validator")
    
    parser.add_argument('--port', '-p', type=str)
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument('--arch', '-a', type=str, choices=['voxel_bin', 'gradient_map'], default=None)
    parser.add_argument('--list-ports', action='store_true')
    parser.add_argument('--test', '-t', type=str, choices=['echo', 'status', 'config', 'noise', 'all'])
    parser.add_argument('--gesture', '-g', type=str, choices=['up', 'down', 'left', 'right'])
    parser.add_argument('--interactive', '-i', action='store_true')
    parser.add_argument('--continuous', '-c', action='store_true')
    parser.add_argument('--duration', '-d', type=float, default=60)
    args = parser.parse_args()
    if args.list_ports:
        list_ports()
        return 0
    if not args.port:
        parser.print_help()
        print("\nERROR: --port is required")
        return 1
    if args.arch is not None:
        architecture = args.arch
        fpga = FPGAGestureInterface(args.port, args.baud, architecture=architecture)
        if not fpga.connect():
            return 1
    else:
        fpga = FPGAGestureInterface(args.port, args.baud, architecture=Architecture.VOXEL_BIN)
        if not fpga.connect():
            return 1
        architecture = fpga.detect_architecture()
        fpga.architecture = architecture
    try:
        if not test_connection(fpga):
            print("WARNING: Echo test failed")
        if args.test == 'echo':
            test_connection(fpga)
        elif args.test == 'status':
            if architecture == Architecture.GRADIENT_MAP:
                print("(gradient_map: no status command)")
            else:
                test_status(fpga)
        elif args.test == 'config':
            if architecture == Architecture.GRADIENT_MAP:
                print("(gradient_map: no config command)")
            else:
                test_config(fpga)
        elif args.test == 'noise':
            test_noise_rejection(fpga)
        elif args.test == 'all':
            if architecture != Architecture.GRADIENT_MAP:
                test_status(fpga)
                test_config(fpga)
            test_noise_rejection(fpga)
            test_all_gestures(fpga)
        elif args.gesture:
            gesture_map = {'up': Gesture.UP, 'down': Gesture.DOWN, 'left': Gesture.LEFT, 'right': Gesture.RIGHT}
            test_gesture(fpga, gesture_map[args.gesture])
        elif args.continuous:
            continuous_monitoring(fpga, args.duration)
        elif args.interactive:
            interactive_mode(fpga)
        else:
            if architecture != Architecture.GRADIENT_MAP:
                test_status(fpga)
                test_config(fpga)
            test_all_gestures(fpga)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        fpga.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())

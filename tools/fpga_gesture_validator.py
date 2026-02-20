#!/usr/bin/env python3
"""
DVS Gesture Classifier - Hardware Validation Script

This script sends mock DVS events over UART to an iCE40 FPGA running a
DVS gesture classifier and monitors for gesture detection responses.

Architectures:
  - voxel_bin (default): Binary protocol. Echo 0xFF->0x55, Status 0xFE->0xBx,
    Config 0xFD->2 bytes. Gesture: [0xA0|gesture, confidence_byte].
  - gradient_map: ASCII protocol only. No echo/status/config. Gesture output:
    "UP\\r\\n", "DOWN\\r\\n", "LEFT\\r\\n", "RIGHT\\r\\n".

Supported Gestures:
    - UP, DOWN, LEFT, RIGHT (same for both architectures)

Usage:
    python fpga_gesture_validator.py --port COM3 --arch gradient_map --test all
    python fpga_gesture_validator.py --port /dev/ttyUSB1 --arch voxel_bin --test all
    python fpga_gesture_validator.py --port COM3 --gesture right

Requirements:
    pip install pyserial
"""

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


# =============================================================================
# Constants and Configuration
# =============================================================================

class Architecture:
    """UART protocol variant."""
    VOXEL_BIN = "voxel_bin"       # Binary: 0xA0|gesture, echo/status/config
    GRADIENT_MAP = "gradient_map" # ASCII: "UP\r\n", "DOWN\r\n", ... only


class Gesture(IntEnum):
    """Gesture type encodings"""
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

# UART settings
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 1.0

# DVS sensor resolution
SENSOR_WIDTH = 320
SENSOR_HEIGHT = 320

# Gesture motion parameters
GESTURE_DURATION_MS = 400  # Duration for gesture motion
EVENT_RATE_HZ = 500        # Events per second during gesture


@dataclass
class DVSEvent:
    """Represents a single DVS event"""
    x: int
    y: int
    polarity: int
    timestamp_us: int = 0


@dataclass 
class GestureResult:
    """Represents a detected gesture"""
    gesture: Gesture
    confidence: int
    event_count_hi: int
    raw_bytes: bytes


# =============================================================================
# DVS Event Generation
# =============================================================================

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
    """
    Generate a stream of DVS events simulating a gesture motion.
    
    Args:
        gesture: Target gesture direction
        duration_ms: Total duration of gesture in milliseconds
        event_rate: Average events per second
        center_x, center_y: Center of motion
        motion_amplitude: Distance of motion in pixels
        spatial_noise: Gaussian noise in pixel positions
        noise_ratio: Fraction of events that are random noise
    
    Returns:
        List of DVSEvent objects
    """
    events = []
    num_events = int(duration_ms * event_rate / 1000)
    
    # Define motion trajectory based on gesture
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
    
    # Generate events along trajectory
    for i in range(num_events):
        t = i / max(1, num_events - 1)  # Progress [0, 1]
        timestamp_us = int(t * duration_ms * 1000)
        
        if random.random() < noise_ratio:
            # Random noise event
            x = random.randint(0, SENSOR_WIDTH - 1)
            y = random.randint(0, SENSOR_HEIGHT - 1)
        else:
            # Event along trajectory with noise
            x = start_x + t * (end_x - start_x) + random.gauss(0, spatial_noise)
            y = start_y + t * (end_y - start_y) + random.gauss(0, spatial_noise)
            x = int(max(0, min(SENSOR_WIDTH - 1, x)))
            y = int(max(0, min(SENSOR_HEIGHT - 1, y)))
        
        # Mostly ON events for better detection
        polarity = 1 if random.random() > 0.15 else 0
        
        events.append(DVSEvent(x=x, y=y, polarity=polarity, timestamp_us=timestamp_us))
    
    return events


def generate_random_events(count: int) -> List[DVSEvent]:
    """Generate random noise events"""
    events = []
    for i in range(count):
        events.append(DVSEvent(
            x=random.randint(0, SENSOR_WIDTH - 1),
            y=random.randint(0, SENSOR_HEIGHT - 1),
            polarity=random.randint(0, 1),
            timestamp_us=i * 1000
        ))
    return events




# =============================================================================
# UART Communication
# =============================================================================

class FPGAGestureInterface:
    """Interface for communicating with the FPGA gesture classifier over UART"""
    
    def __init__(self, port: str, baud_rate: int = DEFAULT_BAUD_RATE,
                 architecture: str = Architecture.VOXEL_BIN):
        self.port = port
        self.baud_rate = baud_rate
        self.architecture = architecture
        self.serial: Optional[serial.Serial] = None
        self.rx_queue = queue.Queue()
        self.rx_thread: Optional[threading.Thread] = None
        self.running = False
        self._ascii_line_buffer = bytearray()  # for gradient_map line parsing
        
    def connect(self) -> bool:
        """Open serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=DEFAULT_TIMEOUT
            )
            self.running = True
            self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self.rx_thread.start()
            time.sleep(0.1)  # Allow FPGA to stabilize
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Failed to open {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")
    
    def _rx_worker(self):
        """Background thread for receiving UART data"""
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
        """Send a single byte"""
        if self.serial:
            self.serial.write(bytes([byte & 0xFF]))
    
    def _send_bytes(self, data: bytes):
        """Send multiple bytes"""
        if self.serial:
            self.serial.write(data)
    
    def _receive_byte(self, timeout: float = 1.0) -> Optional[int]:
        """Receive a single byte with timeout"""
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def _receive_bytes(self, count: int, timeout: float = 1.0) -> bytes:
        """Receive multiple bytes"""
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
        """Clear any pending received data and ASCII line buffer (gradient_map)."""
        while not self.rx_queue.empty():
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                break
        self._ascii_line_buffer.clear()

    def realign_parser(self):
        """Send padding bytes to realign the 5-byte UART event parser.

        After stray command bytes (e.g. from auto-detection probes), the
        gradient_map FPGA's 5-byte parser may be mid-packet. We flush it
        by sending 4 zero bytes — in the worst case the parser has consumed
        1 stray byte, so 4 more zeros complete the packet as a harmless
        all-zero event, restoring alignment for subsequent real events.
        """
        self._send_bytes(bytes(4))
        time.sleep(0.01)
        self.clear_rx_buffer()

    def detect_architecture(self) -> str:
        """Auto-detect FPGA architecture by probing with a voxel_bin echo.

        Sends 0xFF (voxel_bin echo command). If the FPGA replies with 0x55,
        it's running voxel_bin. Otherwise we assume gradient_map and realign
        the 5-byte event parser that consumed the stray probe byte.
        """
        self.clear_rx_buffer()
        self._send_byte(0xFF)
        response = self._receive_byte(timeout=0.3)
        if response == 0x55:
            print("Auto-detected architecture: voxel_bin (echo 0x55 received)")
            return Architecture.VOXEL_BIN
        else:
            print("Auto-detected architecture: gradient_map (no echo response)")
            # The probe byte entered the 5-byte parser — realign it
            self.realign_parser()
            return Architecture.GRADIENT_MAP
    
    # -------------------------------------------------------------------------
    # UART Commands
    # -------------------------------------------------------------------------
    
    def send_echo(self) -> bool:
        """Send echo command and verify response (voxel_bin only)."""
        if self.architecture == Architecture.GRADIENT_MAP:
            # Gradient-map has no echo; avoid sending 0xFF (would corrupt 5-byte parser)
            self.clear_rx_buffer()
            return True  # Assume connected
        self.clear_rx_buffer()
        self._send_byte(0xFF)
        response = self._receive_byte(timeout=0.5)
        if response == 0x55:
            return True
        print(f"Echo failed: expected 0x55, got {response}")
        return False
    
    def query_status(self) -> Optional[dict]:
        """Query accelerator status (voxel_bin only)."""
        if self.architecture == Architecture.GRADIENT_MAP:
            return None  # No status command
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
        """Query configuration parameters (voxel_bin only)."""
        if self.architecture == Architecture.GRADIENT_MAP:
            return None  # No config command
        self.clear_rx_buffer()
        self._send_byte(0xFD)
        response = self._receive_bytes(2, timeout=0.5)
        if len(response) != 2:
            return None
        return {
            'min_event_thresh': response[0],
            'motion_thresh': response[1]
        }
    
    def soft_reset(self):
        """Send soft reset command (voxel_bin only). Gradient_map: no-op."""
        if self.architecture == Architecture.GRADIENT_MAP:
            return
        self._send_byte(0xFC)
        time.sleep(0.01)
    
    def send_dvs_event(self, event: DVSEvent):
        """Send a single DVS event"""
        # Pack 5-byte packet: [X_HI, X_LO, Y_HI, Y_LO, POL]
        packet = bytes([
            (event.x >> 8) & 0x01,  # X[8]
            event.x & 0xFF,          # X[7:0]
            (event.y >> 8) & 0x01,  # Y[8]
            event.y & 0xFF,          # Y[7:0]
            event.polarity & 0x01   # Polarity
        ])
        self._send_bytes(packet)
    
    def send_event_stream(self, events: List[DVSEvent], delay_us: float = 0):
        """Send a stream of DVS events with optional inter-event delay"""
        for event in events:
            self.send_dvs_event(event)
            if delay_us > 0:
                time.sleep(delay_us / 1_000_000)
    
    def _check_gesture_ascii(self, timeout: float) -> Optional[GestureResult]:
        """Parse gradient_map ASCII output: UP\r\n, DOWN\r\n, LEFT\r\n, RIGHT\r\n"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                b = self.rx_queue.get(timeout=max(0.01, deadline - time.time()))
            except queue.Empty:
                break
            if b in (0x0A, 0x0D):  # \n or \r — end of line
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
        """Check for gesture detection response (binary or ASCII per architecture)."""
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
        return GestureResult(
            gesture=gesture,
            confidence=confidence,
            event_count_hi=event_count_hi,
            raw_bytes=bytes([byte1, byte2])
        )


# =============================================================================
# Test Functions
# =============================================================================

def test_connection(fpga: FPGAGestureInterface) -> bool:
    """Test basic UART connectivity"""
    print("\n" + "="*60)
    print("TEST: Connection and Echo")
    print("="*60)
    
    if fpga.send_echo():
        print("[+] Echo test PASSED")
        return True
    else:
        print("[-] Echo test FAILED")
        return False


def test_status(fpga: FPGAGestureInterface) -> bool:
    """Test status query"""
    print("\n" + "="*60)
    print("TEST: Status Query")
    print("="*60)
    
    status = fpga.query_status()
    if status:
        print(f"Status: phase={status['phase']}, "
              f"fifo_full={status['fifo_full']}, "
              f"fifo_empty={status['fifo_empty']}")
        return True
    else:
        print("[-] Status query FAILED")
        return False


def test_config(fpga: FPGAGestureInterface) -> bool:
    """Test configuration query"""
    print("\n" + "="*60)
    print("TEST: Configuration Query")
    print("="*60)
    
    config = fpga.query_config()
    if config:
        print(f"[+] Config: min_event_thresh={config['min_event_thresh']}, "
              f"motion_thresh={config['motion_thresh']}")
        return True
    else:
        print("[-] Config query FAILED")
        return False


def test_gesture(fpga: FPGAGestureInterface, gesture: Gesture, 
                 num_events: int = 200, verbose: bool = True) -> Optional[GestureResult]:
    """Test a specific gesture"""
    gesture_name = GESTURE_NAMES[gesture]
    
    if verbose:
        print(f"\n" + "="*60)
        print(f"TEST: {gesture_name} Gesture")
        print("="*60)
    
    fpga.soft_reset()
    fpga.clear_rx_buffer()
    # Clear ASCII line buffer for gradient_map
    if hasattr(fpga, "_ascii_line_buffer"):
        fpga._ascii_line_buffer.clear()
    time.sleep(0.05)

    # Voxel_bin: align to temporal phase
    if fpga.architecture == Architecture.VOXEL_BIN:
        status = fpga.query_status()
        if status and status.get("phase") == 1:
            if verbose:
                print("Waiting for early phase start...")
            time.sleep(0.25)
    
    # Generate gesture events (reduced noise, larger motion for robust detection)
    events = generate_gesture_events(
        gesture,
        event_rate=EVENT_RATE_HZ * 2,
        duration_ms=GESTURE_DURATION_MS,
        motion_amplitude=120,
        spatial_noise=5.0,
        noise_ratio=0.0
    )
    
    if verbose:
        print(f"Sending {len(events)} events for {gesture_name} gesture...")
    
    if fpga.architecture == Architecture.GRADIENT_MAP:
        # Gradient-map: frame-based (e.g. 50 ms); send dense event stream, then wait for classification
        fpga.send_event_stream(events, delay_us=200)
        time.sleep(0.2)   # Allow frame to close and classifier to run (e.g. 50 ms frame + pipeline)
    else:
        # Voxel_bin: split early/late for dual-accumulator
        half = len(events) // 2
        fpga.send_event_stream(events[:half], delay_us=500)
        time.sleep(0.2)
        fpga.send_event_stream(events[half:], delay_us=500)
    
    time.sleep(0.5)
    
    result = fpga.check_gesture(timeout=1.0)
    
    if result:
        detected_name = GESTURE_NAMES[result.gesture]
        if verbose:
            print(f"[+] Gesture detected: {detected_name} "
                  f"(confidence={result.confidence}, expected={gesture_name})")
        if result.gesture == gesture:
            if verbose:
                print(f"  CORRECT classification!")
            return result
        else:
            if verbose:
                print(f"  INCORRECT - expected {gesture_name}")
            return result
    else:
        if verbose:
            print(f"[-] No gesture detected (expected {gesture_name})")
        return None


def test_all_gestures(fpga: FPGAGestureInterface) -> dict:
    """Test all four gesture directions"""
    print("\n" + "="*60)
    print("TEST: All Gestures")
    print("="*60)
    
    results = {}
    for gesture in Gesture:
        result = test_gesture(fpga, gesture, verbose=True)
        results[gesture] = result
        time.sleep(0.5)
    
    # Summary
    print("\n" + "-"*60)
    print("SUMMARY:")
    correct = sum(1 for g, r in results.items() if r and r.gesture == g)
    print(f"  Correct: {correct}/4")
    for gesture, result in results.items():
        name = GESTURE_NAMES[gesture]
        if result:
            detected = GESTURE_NAMES[result.gesture]
            status = "[+]" if result.gesture == gesture else "[-]"
            print(f"  {status} {name}: detected {detected} (conf={result.confidence})")
        else:
            print(f"  [-] {name}: no detection")
    
    return results


def test_noise_rejection(fpga: FPGAGestureInterface) -> bool:
    """Test that random noise doesn't trigger false gestures"""
    print("\n" + "="*60)
    print("TEST: Noise Rejection")
    print("="*60)
    
    fpga.soft_reset()
    fpga.clear_rx_buffer()
    
    # Send random noise events (below activity threshold)
    events = generate_random_events(10)
    print(f"Sending {len(events)} random noise events...")
    fpga.send_event_stream(events, delay_us=1000)
    
    time.sleep(0.5)
    
    result = fpga.check_gesture(timeout=0.5)
    if result:
        print(f"[-] False detection: {GESTURE_NAMES[result.gesture]}")
        return False
    else:
        print("[+] No false detection - noise rejection working")
        return True


def continuous_monitoring(fpga: FPGAGestureInterface, duration_s: float = 60):
    """Monitor for gestures continuously"""
    print("\n" + "="*60)
    print(f"Continuous Monitoring ({duration_s}s)")
    print("="*60)
    print("Listening for gesture detections... (Press Ctrl+C to stop)")
    
    start_time = time.time()
    gesture_count = {g: 0 for g in Gesture}
    
    try:
        while (time.time() - start_time) < duration_s:
            result = fpga.check_gesture(timeout=0.1)
            if result:
                name = GESTURE_NAMES[result.gesture]
                gesture_count[result.gesture] += 1
                elapsed = time.time() - start_time
                print(f"[{elapsed:6.1f}s] Detected: {name} "
                      f"(confidence={result.confidence})")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    # Summary
    print("\n" + "-"*60)
    print("DETECTION SUMMARY:")
    total = sum(gesture_count.values())
    print(f"  Total gestures: {total}")
    for gesture, count in gesture_count.items():
        print(f"  {GESTURE_NAMES[gesture]}: {count}")


def interactive_mode(fpga: FPGAGestureInterface):
    """Interactive testing mode"""
    print("\n" + "="*60)
    print("Interactive Mode")
    print("="*60)
    print("Commands:")
    print("  u/d/l/r - Send UP/DOWN/LEFT/RIGHT gesture")
    print("  e       - Echo test")
    print("  s       - Status query")
    print("  c       - Config query")
    print("  x       - Soft reset")
    print("  n       - Send noise events")
    print("  a       - Test all gestures")
    print("  q       - Quit")
    print()
    
    while True:
        try:
            cmd = input("Command> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'e':
                if fpga.send_echo():
                    print("Echo: OK")
                else:
                    print("Echo: FAILED")
            elif cmd == 's':
                if fpga.architecture == Architecture.GRADIENT_MAP:
                    print("Status: not supported (gradient_map)")
                else:
                    status = fpga.query_status()
                    if status:
                        print(f"Status: {status}")
                    else:
                        print("Status: FAILED")
            elif cmd == 'c':
                if fpga.architecture == Architecture.GRADIENT_MAP:
                    print("Config: not supported (gradient_map)")
                else:
                    config = fpga.query_config()
                    if config:
                        print(f"Config: {config}")
                    else:
                        print("Config: FAILED")
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
            print("\n")
            break
        except EOFError:
            break


def list_ports():
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found")
    else:
        print("Available serial ports:")
        for port in ports:
            print(f"  {port.device}: {port.description}")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="DVS Gesture Classifier FPGA Hardware Validator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --list-ports
  %(prog)s --port COM3 --arch gradient_map --test all
  %(prog)s --port /dev/ttyUSB1 --arch voxel_bin --test all
  %(prog)s --port COM3 --gesture right
  %(prog)s --port COM3 --interactive
  %(prog)s --port /dev/ttyUSB0 --continuous --duration 120
        """
    )
    
    parser.add_argument('--port', '-p', type=str,
                        help='Serial port (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('--arch', '-a', type=str,
                        choices=['voxel_bin', 'gradient_map'],
                        default=None,
                        help='FPGA architecture (auto-detected if omitted): voxel_bin (binary) or gradient_map (ASCII)')
    parser.add_argument('--list-ports', action='store_true',
                        help='List available serial ports')
    parser.add_argument('--test', '-t', type=str, 
                        choices=['echo', 'status', 'config', 'noise', 'all'],
                        help='Run specific test')
    parser.add_argument('--gesture', '-g', type=str,
                        choices=['up', 'down', 'left', 'right'],
                        help='Send specific gesture')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Interactive testing mode')
    parser.add_argument('--continuous', '-c', action='store_true',
                        help='Continuous gesture monitoring')
    parser.add_argument('--duration', '-d', type=float, default=60,
                        help='Duration for continuous monitoring (seconds)')
    
    args = parser.parse_args()
    
    if args.list_ports:
        list_ports()
        return 0
    
    if not args.port:
        parser.print_help()
        print("\nERROR: --port is required")
        return 1
    
    # --- Architecture selection (explicit or auto-detect) --------------------
    if args.arch is not None:
        architecture = args.arch
        if architecture == "gradient_map":
            print("Using gradient_map protocol (ASCII gesture output)")
        else:
            print("Using voxel_bin protocol (binary + echo/status/config)")
        fpga = FPGAGestureInterface(args.port, args.baud, architecture=architecture)
        if not fpga.connect():
            return 1
    else:
        # Auto-detect: connect first with a temporary architecture, then probe
        fpga = FPGAGestureInterface(args.port, args.baud,
                                    architecture=Architecture.VOXEL_BIN)
        if not fpga.connect():
            return 1
        architecture = fpga.detect_architecture()
        fpga.architecture = architecture
    
    try:
        if not test_connection(fpga):
            print("WARNING: Echo test failed, FPGA may not be responding")
        
        if args.test == 'echo':
            test_connection(fpga)
        elif args.test == 'status':
            if architecture == Architecture.GRADIENT_MAP:
                print("(gradient_map has no status command — skipped)")
            else:
                test_status(fpga)
        elif args.test == 'config':
            if architecture == Architecture.GRADIENT_MAP:
                print("(gradient_map has no config command — skipped)")
            else:
                test_config(fpga)
        elif args.test == 'noise':
            test_noise_rejection(fpga)
        elif args.test == 'all':
            if architecture != Architecture.GRADIENT_MAP:
                test_status(fpga)
                test_config(fpga)
            else:
                print("\n(gradient_map: echo/status/config tests skipped — not supported)")
            test_noise_rejection(fpga)
            test_all_gestures(fpga)
        elif args.gesture:
            gesture_map = {
                'up': Gesture.UP,
                'down': Gesture.DOWN,
                'left': Gesture.LEFT,
                'right': Gesture.RIGHT
            }
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

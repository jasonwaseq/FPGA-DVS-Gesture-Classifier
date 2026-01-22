#!/usr/bin/env python3
"""
Ice40 Validation Script for DVS Gesture Accelerator
Sends mock DVS events via UART and verifies gesture classification

UART Protocol:
  TX to FPGA: 4 bytes per event [X, Y, POL, TS]
  RX from FPGA: 1 byte [0xAG] where G = gesture (0-3)

NOTE: Design uses internal power-on reset - no button needed!
Just program the bitstream and it starts running immediately.

Test Mode: Send 0xFF to receive 0x55 (verifies UART working)
"""

import sys
import serial
import time
import argparse
import threading

GESTURES = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}


class GestureCapture:
    """Thread-safe gesture capture from serial port"""
    def __init__(self, ser):
        self.ser = ser
        self.captured = None
        self.running = False
        self.thread = None
    
    def start(self):
        """Start capturing in background"""
        self.captured = None
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop capturing"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=0.5)
    
    def _capture_loop(self):
        """Background capture loop"""
        while self.running:
            if self.ser.in_waiting > 0:
                data = self.ser.read(1)
                if data:
                    byte = data[0]
                    # Check for gesture marker (0xAx)
                    if (byte & 0xF0) == 0xA0:
                        self.captured = byte & 0x03
                        return
            time.sleep(0.001)
    
    def get_result(self, timeout=1.0):
        """Wait for and return captured gesture"""
        start = time.time()
        while time.time() - start < timeout:
            if self.captured is not None:
                return self.captured
            time.sleep(0.01)
        return None


def send_event(ser, x, y, polarity, ts=0):
    """Send a single DVS event (4 bytes)"""
    packet = bytes([x & 0x7F, y & 0x7F, polarity & 0x01, ts & 0xFF])
    ser.write(packet)


def generate_gesture_events(direction, num_events=100):
    """Generate DVS events simulating a gesture movement"""
    events = []
    cx, cy = 64, 64  # Center
    
    for i in range(num_events):
        p = i / num_events  # Progress 0 to 1
        
        if direction == 'UP':
            x = cx + (i % 10) - 5
            y = cy + int(p * 50)
        elif direction == 'DOWN':
            x = cx + (i % 10) - 5
            y = cy - int(p * 50)
        elif direction == 'LEFT':
            x = cx - int(p * 50)
            y = cy + (i % 10) - 5
        elif direction == 'RIGHT':
            x = cx + int(p * 50)
            y = cy + (i % 10) - 5
        else:
            x, y = cx, cy
        
        # Clamp to 0-127
        x = max(0, min(127, x))
        y = max(0, min(127, y))
        events.append((x, y, 1, i % 256))
    
    return events



def test_uart_echo(ser):
    """Test basic UART connectivity by sending 0xFF and expecting 0x55 back"""
    print("  Testing UART echo...", end=' ', flush=True)
    
    ser.reset_input_buffer()
    ser.write(bytes([0xFF]))
    ser.flush()
    
    time.sleep(0.1)
    
    if ser.in_waiting > 0:
        response = ser.read(1)
        if response and response[0] == 0x55:
            print("PASS (got 0x55)")
            return True
        else:
            print(f"FAIL (got 0x{response[0]:02X}, expected 0x55)")
            return False
    else:
        print("FAIL (no response)")
        return False


def test_gesture(ser, direction, expected, num_events=100, verbose=False):
    """Send gesture events and check result"""
    print(f"  Testing {direction}...", end=' ', flush=True)
    
    # Clear any pending data
    ser.reset_input_buffer()
    
    # Start capturing responses in background
    capture = GestureCapture(ser)
    capture.start()
    
    # Generate and send events
    events = generate_gesture_events(direction, num_events)
    for x, y, pol, ts in events:
        send_event(ser, x, y, pol, ts)
        # Small delay to not overflow FPGA UART buffer
        time.sleep(0.0005)
    
    # Wait for result
    result = capture.get_result(timeout=1.0)
    capture.stop()
    
    if result is None:
        print("FAIL (no response)")
        return False
    
    detected = GESTURES.get(result, f'UNKNOWN({result})')
    if result == expected:
        print(f"PASS -> {detected}")
        return True
    else:
        print(f"FAIL -> {detected} (expected {direction})")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Ice40 DVS Gesture Accelerator Validation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python validate_ice40.py /dev/ttyUSB0
  python validate_ice40.py COM3 -v
  python validate_ice40.py /dev/ttyACM0 -n 200
''')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('-n', '--num-events', type=int, default=100, help='Events per gesture (default: 100)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--reset', action='store_true', help='Toggle DTR to reset FPGA')
    args = parser.parse_args()
    
    print(f"Connecting to {args.port} @ {args.baud} baud...")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(0.5)  # Wait for connection to stabilize
        
        if args.reset:
            print("Resetting FPGA via DTR...")
            ser.dtr = False
            time.sleep(0.1)
            ser.dtr = True
            time.sleep(0.5)
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("\n=== DVS Gesture Accelerator Test ===\n")
        
        # First test basic UART connectivity
        if not test_uart_echo(ser):
            print("\n  UART echo test failed!")
            print("  Check: 1) FPGA is programmed")
            print("         2) Correct serial port")
            print("         3) LED heartbeat is blinking")
            print("         4) Reset button not pressed\n")
            ser.close()
            return 1
        
        print()  # Blank line after echo test
        
        tests = [
            ('RIGHT', 3),
            ('LEFT', 2),
            ('UP', 0),
            ('DOWN', 1),
        ]
        
        passed = 0
        for direction, expected_code in tests:
            if test_gesture(ser, direction, expected_code, args.num_events, args.verbose):
                passed += 1
            time.sleep(0.2)
        
        print(f"\n=== Results: {passed}/{len(tests)} passed ===\n")
        
        ser.close()
        return 0 if passed == len(tests) else 1
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nAborted")
        return 1


if __name__ == '__main__':
    sys.exit(main())

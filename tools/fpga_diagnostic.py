#!/usr/bin/env python3
"""Diagnostic: test DVS event injection and FPGA gesture response."""

import time
import sys
from pathlib import Path
import serial.tools.list_ports

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.fpga_gesture_validator import FPGAGestureInterface, Gesture, GESTURE_NAMES

def auto_detect_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc_lower = port.description.lower()
        if any(kw in desc_lower for kw in ['ftdi', 'usb', 'ice']):
            return port.device
    return None if not ports else ports[0].device

def test_basic_uart(fpga):
    if fpga.send_echo():
        print("Echo OK")
    else:
        print("Echo FAILED")
        return False
    status = fpga.query_status()
    if status:
        print(f"Status: state={status['state']}, fifo_empty={status['fifo_empty']}")
    else:
        print("Status FAILED")
        return False
    return True

def test_fifo_operation(fpga):
    for i in range(50):
        x = 100 + (i % 50)
        y = 100 + (i // 50) * 10
        fpga.send_dvs_event(__import__('tools.fpga_gesture_validator', fromlist=['DVSEvent']).DVSEvent(x, y, 1))
    time.sleep(0.5)
    if fpga.send_echo():
        print("FPGA responsive after event injection")
        return True
    else:
        print("FPGA unresponsive - FIFO may be stuck")
        return False

def test_window_timing(fpga):
    test_cases = [
        (0.05, "50ms"),
        (0.1, "100ms"),
        (0.2, "200ms"),
        (0.4, "400ms"),
        (0.45, "450ms"),
    ]
    for wait_time, description in test_cases:
        fpga.soft_reset()
        fpga.clear_rx_buffer()
        time.sleep(wait_time)
        from tools.fpga_gesture_validator import generate_gesture_events, EVENT_RATE_HZ, GESTURE_DURATION_MS
        events = generate_gesture_events(Gesture.RIGHT, event_rate=EVENT_RATE_HZ * 2, duration_ms=GESTURE_DURATION_MS)
        print(f"After {description}: Sending {len(events)} events...")
        fpga.send_event_stream(events[:len(events)//2], delay_us=100)
        time.sleep(0.200)
        fpga.send_event_stream(events[len(events)//2:], delay_us=100)
        time.sleep(0.400)
        result = fpga.check_gesture(timeout=0.5)
        if result:
            print(f"  Detected: {GESTURE_NAMES[result.gesture]}")
        else:
            print(f"  No detection")

def test_motion_computation(fpga):
    fpga.soft_reset()
    fpga.clear_rx_buffer()
    time.sleep(0.450)
    from tools.fpga_gesture_validator import generate_gesture_events, EVENT_RATE_HZ, GESTURE_DURATION_MS
    events = generate_gesture_events(Gesture.RIGHT, event_rate=EVENT_RATE_HZ * 4,
                                     duration_ms=GESTURE_DURATION_MS, motion_amplitude=150,
                                     spatial_noise=5.0, noise_ratio=0.01)
    print(f"Sending {len(events)} high-amplitude RIGHT gesture events...")
    half = len(events) // 2
    fpga.send_event_stream(events[:half], delay_us=50)
    time.sleep(0.220)
    fpga.send_event_stream(events[half:], delay_us=50)
    time.sleep(0.220)
    result = fpga.check_gesture(timeout=2.0)
    if result:
        print(f"Detected: {GESTURE_NAMES[result.gesture]}")
    else:
        print(f"No detection")

def main():
    port = auto_detect_port()
    if not port:
        print("ERROR: Could not find FPGA port")
        return 1
    
    print(f"Connecting to {port}...")
    fpga = FPGAGestureInterface(port)
    if not fpga.connect():
        return 1
    
    try:
        if not test_basic_uart(fpga):
            print("UART communication failed")
            return 1
        if not test_fifo_operation(fpga):
            print("FIFO not responding to events")
            return 1
        test_window_timing(fpga)
        test_motion_computation(fpga)
        print("Diagnostic Complete")
    finally:
        fpga.disconnect()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

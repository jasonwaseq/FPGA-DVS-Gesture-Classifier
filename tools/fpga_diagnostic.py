#!/usr/bin/env python3
"""
Diagnostic script to test DVS event injection and FPGA response
Helps debug why gestures aren't being detected
"""

import time
import sys
import serial.tools.list_ports
from tools.fpga_gesture_validator import FPGAGestureInterface, Gesture, GESTURE_NAMES

def auto_detect_port():
    """Find FPGA serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc_lower = port.description.lower()
        if any(kw in desc_lower for kw in ['ftdi', 'usb', 'ice']):
            return port.device
    return None if not ports else ports[0].device

def test_basic_uart(fpga):
    """Test basic UART communication"""
    print("\n" + "="*60)
    print("DIAGNOSTIC: Basic UART Communication")
    print("="*60)
    
    # Test echo
    print("  Testing echo (send 0xFF, expect 0x55)...")
    if fpga.send_echo():
        print("  ✓ Echo works")
    else:
        print("  ✗ Echo FAILED - FPGA not responding")
        return False
    
    # Test status
    print("  Querying status...")
    status = fpga.query_status()
    if status:
        print(f"  ✓ Status: state={status['state']}, fifo_empty={status['fifo_empty']}")
    else:
        print("  ✗ Status FAILED")
        return False
    
    return True

def test_fifo_operation(fpga):
    """Test event injection into FIFO"""
    print("\n" + "="*60)
    print("DIAGNOSTIC: FIFO Event Injection")
    print("="*60)
    
    # Send a small burst of events
    num_events = 50
    print(f"  Sending {num_events} test events...")
    
    for i in range(num_events):
        x = 100 + (i % 50)
        y = 100 + (i // 50) * 10
        fpga.send_dvs_event(__import__('tools.fpga_gesture_validator', fromlist=['DVSEvent']).DVSEvent(x, y, 1))
    
    # Wait for processing
    time.sleep(0.5)
    
    # Check if system still responds
    print("  Checking if system is still responsive...")
    if fpga.send_echo():
        print("  ✓ FPGA responsive after event injection")
        return True
    else:
        print("  ✗ FPGA unresponsive - FIFO may be stuck")
        return False

def test_window_timing(fpga):
    """Verify window phase timing"""
    print("\n" + "="*60)
    print("DIAGNOSTIC: Window Phase Timing")
    print("="*60)
    
    print("  Testing various phase offsets...")
    
    # Test different waits to understand window phase
    test_cases = [
        (0.05, "50ms (very short)"),
        (0.1, "100ms"),
        (0.2, "200ms (one phase)"),
        (0.4, "400ms (full window)"),
        (0.45, "450ms (full window + margin)"),
    ]
    
    for wait_time, description in test_cases:
        fpga.soft_reset()
        fpga.clear_rx_buffer()
        time.sleep(wait_time)
        
        # Send a test gesture and immediately check for response
        from tools.fpga_gesture_validator import generate_gesture_events, EVENT_RATE_HZ, GESTURE_DURATION_MS
        events = generate_gesture_events(Gesture.RIGHT, event_rate=EVENT_RATE_HZ * 2, duration_ms=GESTURE_DURATION_MS)
        
        print(f"    After {description}: Sending {len(events)} events...")
        fpga.send_event_stream(events[:len(events)//2], delay_us=100)
        time.sleep(0.200)
        fpga.send_event_stream(events[len(events)//2:], delay_us=100)
        time.sleep(0.400)
        
        result = fpga.check_gesture(timeout=0.5)
        if result:
            print(f"    ✓ Detected: {GESTURE_NAMES[result.gesture]}")
        else:
            print(f"    ✗ No detection")

def test_motion_computation(fpga):
    """Test if motion is actually being computed"""
    print("\n" + "="*60)
    print("DIAGNOSTIC: Motion Vector Computation")
    print("="*60)
    
    print("  This requires examining the actual motion vectors...")
    print("  Sending large amplitude gesture for clear motion...")
    
    fpga.soft_reset()
    fpga.clear_rx_buffer()
    time.sleep(0.450)
    
    from tools.fpga_gesture_validator import generate_gesture_events, EVENT_RATE_HZ, GESTURE_DURATION_MS
    
    # Generate with maximum amplitude
    events = generate_gesture_events(Gesture.RIGHT, 
                                     event_rate=EVENT_RATE_HZ * 4,  # Fastest injection
                                     duration_ms=GESTURE_DURATION_MS,
                                     motion_amplitude=150,  # Maximum motion
                                     spatial_noise=5.0,     # Minimum noise
                                     noise_ratio=0.01)      # Almost no noise
    
    print(f"  Sending {len(events)} high-amplitude RIGHT gesture events...")
    half = len(events) // 2
    fpga.send_event_stream(events[:half], delay_us=50)
    time.sleep(0.220)
    fpga.send_event_stream(events[half:], delay_us=50)
    time.sleep(0.220)
    
    result = fpga.check_gesture(timeout=2.0)
    if result:
        print(f"  ✓ Detected: {GESTURE_NAMES[result.gesture]}")
    else:
        print(f"  ✗ No detection - motion computation may not be working")

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
        # Run diagnostics
        if not test_basic_uart(fpga):
            print("\n✗ UART communication failed - check physical connection")
            return 1
        
        if not test_fifo_operation(fpga):
            print("\n✗ FIFO not responding to events - check HDL")
            return 1
        
        test_window_timing(fpga)
        test_motion_computation(fpga)
        
        print("\n" + "="*60)
        print("Diagnostic Complete")
        print("="*60)
        
    finally:
        fpga.disconnect()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

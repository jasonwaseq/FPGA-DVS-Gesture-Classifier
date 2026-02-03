#!/usr/bin/env python3
"""
DVS Event File Player - Replay saved DVS events to FPGA via UART

This script reads events saved by dvs_camera_emulator.py and replays them
to the FPGA at a configurable rate.

Usage:
    python dvs_event_player.py events.bin                         # Auto-detect port
    python dvs_event_player.py events.bin --port /dev/ttyUSB0
    python dvs_event_player.py events.bin --port /dev/ttyUSB0 --speed 2.0
    python dvs_event_player.py events.bin --preview  # Visualize only
"""

import argparse
import struct
import sys
import time
from typing import List, Tuple

import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


DVS_RESOLUTION = 320


class DVSEvent:
    """DVS Event from file"""
    def __init__(self, x: int, y: int, polarity: bool, timestamp_us: int):
        self.x = x
        self.y = y
        self.polarity = polarity
        self.timestamp_us = timestamp_us
    
    def to_bytes(self) -> bytes:
        """Convert to UART protocol format"""
        x_hi = (self.x >> 8) & 0x01
        x_lo = self.x & 0xFF
        y_hi = (self.y >> 8) & 0x01
        y_lo = self.y & 0xFF
        pol = 1 if self.polarity else 0
        return bytes([x_hi, x_lo, y_hi, y_lo, pol])


def load_events(filename: str) -> List[DVSEvent]:
    """Load events from binary file"""
    events = []
    
    with open(filename, 'rb') as f:
        # Read and verify header
        header = f.read(4)
        if header != b'DVS1':
            print(f"ERROR: Invalid file format (expected DVS1 header)")
            return events
        
        # Read events (9 bytes each: x(2), y(2), polarity(1), timestamp(4))
        while True:
            data = f.read(9)
            if len(data) < 9:
                break
            
            x, y, pol, timestamp = struct.unpack('<HHBI', data)
            event = DVSEvent(x, y, pol == 1, timestamp)
            events.append(event)
    
    return events


def get_event_stats(events: List[DVSEvent]) -> dict:
    """Calculate statistics for event list"""
    if not events:
        return {'count': 0}
    
    on_count = sum(1 for e in events if e.polarity)
    off_count = len(events) - on_count
    
    timestamps = [e.timestamp_us for e in events]
    duration_us = max(timestamps) - min(timestamps)
    
    return {
        'count': len(events),
        'on_events': on_count,
        'off_events': off_count,
        'duration_ms': duration_us / 1000,
        'event_rate': len(events) / (duration_us / 1_000_000) if duration_us > 0 else 0
    }


def visualize_events(
    events: List[DVSEvent],
    window_ms: float = 33.0,
    resolution: int = DVS_RESOLUTION
) -> np.ndarray:
    """Create visualization of events within time window"""
    vis = np.zeros((resolution, resolution, 3), dtype=np.uint8)
    
    for event in events:
        x, y = event.x, event.y
        if 0 <= x < resolution and 0 <= y < resolution:
            if event.polarity:  # ON - blue
                vis[y, x, 0] = 255
            else:  # OFF - red
                vis[y, x, 2] = 255
    
    return vis


def auto_detect_port() -> str:
    """Automatically detect an FPGA serial port.
    
    Returns:
        Port device name if found, None otherwise
    """
    if not HAS_SERIAL:
        return None
    
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        return None
    
    # Prefer common FPGA/FTDI devices
    ftdi_keywords = ['ftdi', 'ft232', 'ft2232', 'usb serial', 'icebreaker', 'ice40']
    
    # First pass: look for FTDI or FPGA-related devices
    for port in ports:
        desc_lower = port.description.lower()
        if any(keyword in desc_lower for keyword in ftdi_keywords):
            print(f"Auto-detected port: {port.device} ({port.description})")
            return port.device
    
    # Second pass: if only one port available, use it
    if len(ports) == 1:
        print(f"Auto-detected port: {ports[0].device} ({ports[0].description})")
        return ports[0].device
    
    # Multiple ports and no clear match
    print("Multiple serial ports found, please specify with --port:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return None


def main():
    parser = argparse.ArgumentParser(
        description='Replay saved DVS events to FPGA via UART'
    )
    parser.add_argument('input', help='Input event file (.bin)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for UART output (auto-detects if not specified)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='UART baud rate')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier')
    parser.add_argument('--preview', action='store_true',
                        help='Show preview (requires OpenCV)')
    parser.add_argument('--loop', action='store_true',
                        help='Loop playback continuously')
    parser.add_argument('--info', action='store_true',
                        help='Show file info and exit')
    
    args = parser.parse_args()
    
    # Load events
    print(f"Loading events from: {args.input}")
    events = load_events(args.input)
    
    if not events:
        print("No events loaded")
        sys.exit(1)
    
    stats = get_event_stats(events)
    print(f"Loaded {stats['count']} events")
    print(f"  ON events:  {stats['on_events']}")
    print(f"  OFF events: {stats['off_events']}")
    print(f"  Duration:   {stats['duration_ms']:.1f} ms")
    print(f"  Event rate: {stats['event_rate']:.0f} events/sec")
    
    if args.info:
        sys.exit(0)
    
    # Auto-detect port if not specified but needed for serial output
    port = args.port
    if port is None and not args.preview:
        # Try to auto-detect when no preview mode (preview-only doesn't need port)
        port = auto_detect_port()
        if port:
            args.port = port
    
    # Open serial port if specified
    ser = None
    if args.port:
        if not HAS_SERIAL:
            print("ERROR: pyserial not installed")
            sys.exit(1)
        
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
            print(f"Opened {args.port} @ {args.baud} baud")
        except serial.SerialException as e:
            print(f"ERROR: {e}")
            sys.exit(1)
    
    # Check preview requirements
    if args.preview and not HAS_CV2:
        print("WARNING: OpenCV not installed, preview disabled")
        args.preview = False
    
    print(f"\nPlaying back at {args.speed}x speed...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            start_time = time.time()
            first_event_ts = events[0].timestamp_us
            events_sent = 0
            gestures = []
            
            # Group events by time window for visualization
            vis_window_us = 33000  # ~30fps
            vis_events = []
            last_vis_time = first_event_ts
            
            for i, event in enumerate(events):
                # Calculate target time
                event_offset_us = event.timestamp_us - first_event_ts
                target_time = start_time + (event_offset_us / 1_000_000) / args.speed
                
                # Wait until target time
                now = time.time()
                if target_time > now:
                    time.sleep(target_time - now)
                
                # Send event
                if ser:
                    ser.write(event.to_bytes())
                    events_sent += 1
                    
                    # Check for gesture responses
                    if ser.in_waiting > 0:
                        response = ser.read(ser.in_waiting)
                        for byte in response:
                            if (byte & 0xF0) == 0xA0:
                                gesture = byte & 0x03
                                names = ['UP', 'DOWN', 'LEFT', 'RIGHT']
                                gestures.append(names[gesture])
                                print(f"\n*** GESTURE: {names[gesture]} ***")
                
                # Visualization
                vis_events.append(event)
                
                if args.preview and (event.timestamp_us - last_vis_time >= vis_window_us or i == len(events) - 1):
                    vis = visualize_events(vis_events)
                    vis_scaled = cv2.resize(vis, None, fx=2, fy=2, 
                                           interpolation=cv2.INTER_NEAREST)
                    
                    # Add progress text
                    progress = (i + 1) / len(events) * 100
                    cv2.putText(vis_scaled, f"Progress: {progress:.1f}%", 
                               (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                               0.5, (0, 255, 0), 1)
                    cv2.putText(vis_scaled, f"Events: {events_sent}", 
                               (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                               0.5, (0, 255, 0), 1)
                    
                    cv2.imshow('DVS Event Player', vis_scaled)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        raise KeyboardInterrupt
                    
                    vis_events = []
                    last_vis_time = event.timestamp_us
            
            print(f"\nPlayback complete: {events_sent} events sent")
            if gestures:
                print(f"Gestures detected: {gestures}")
            
            if not args.loop:
                break
            
            print("\nRestarting playback...")
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    finally:
        if ser:
            ser.close()
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

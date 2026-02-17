#!/usr/bin/env python3
"""
Live EVT Stream to FPGA - Real-time gesture recognition

Reads raw EVT2 events from a GENX320 camera (via STM32) and streams them
to the iCE40 FPGA. Displays recognized gestures in real-time.

Usage:
    python evt_stream_to_fpga.py --dvs /dev/ttyACM0 --fpga /dev/ttyUSB1
    python evt_stream_to_fpga.py --dvs COM5 --fpga COM3  (Windows)
"""

import argparse
import sys
import time
import struct
from collections import deque
from threading import Thread, Event
import serial


# EVT2 format constants
VALID_TYPES = {0, 1, 0xA, 0xE, 0xF}  # Exclude 0x8 (sync words)
EVT2_POLARITY_MASK = 0x1


class EVTDecoder:
    """Decode EVT2 format events from raw byte stream"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.events = deque()
        self.word_count = 0
        self.sync_state = False
        
    def feed_bytes(self, data: bytes):
        """Add raw bytes and attempt to decode events"""
        self.buffer.extend(data)
        
        # Process complete 32-bit words
        while len(self.buffer) >= 4:
            word = int.from_bytes(self.buffer[:4], 'little', signed=False)
            self.buffer = self.buffer[4:]
            self.word_count += 1
            
            event_type = (word >> 28) & 0xF
            
            # Filter out type 0x8 (sync/filler words)
            if event_type == 0x8:
                continue
            
            if event_type not in VALID_TYPES or event_type == 0x8:
                continue
                
            # Extract coordinates from EVT2 format
            # Bits [0]: Polarity
            # Bits [9:1]: X coordinate (9 bits)
            # Bits [18:10]: Y coordinate (9 bits)
            # Bits [27:19]: Timestamp or other data
            # Bits [31:28]: Event type
            
            pol = (word >> 0) & 0x1
            x = (word >> 1) & 0x1FF
            y = (word >> 10) & 0x1FF
            
            # Clamp to valid sensor range (0-319)
            x = min(x, 319)
            y = min(y, 319)
            
            self.events.append((x, y, pol))
    
    def get_events(self, max_count=None):
        """Retrieve decoded events"""
        if max_count is None:
            events = list(self.events)
            self.events.clear()
        else:
            events = [self.events.popleft() for _ in range(min(max_count, len(self.events)))]
        return events


def encode_event_packet(x: int, y: int, polarity: int) -> bytes:
    """Encode event to FPGA UART protocol (5 bytes)"""
    x_hi = (x >> 8) & 0x01
    x_lo = x & 0xFF
    y_hi = (y >> 8) & 0x01
    y_lo = y & 0xFF
    pol = 1 if polarity else 0
    return bytes([x_hi, x_lo, y_hi, y_lo, pol])


def decode_gesture_response(byte1: int, byte2: int) -> str:
    """Decode FPGA gesture response"""
    gesture_id = byte1 & 0x0F
    gestures = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}
    return gestures.get(gesture_id, "UNKNOWN")


def read_dvs_thread(port_name: str, decoder: EVTDecoder, stop_event: Event):
    """Background thread to read from DVS camera"""
    try:
        port = serial.Serial(port_name, 115200, timeout=0.1)
        print(f"[DVS] Connected to {port_name}")
        
        while not stop_event.is_set():
            try:
                data = port.read(1024)
                if data:
                    decoder.feed_bytes(data)
            except Exception as e:
                print(f"[DVS] Error reading: {e}")
                time.sleep(0.1)
        
        port.close()
    except Exception as e:
        print(f"[DVS] Failed to open {port_name}: {e}")


def read_fpga_responses(port: serial.Serial):
    """Non-blocking read of FPGA responses"""
    responses = []
    try:
        data = port.read(100)
        if len(data) >= 2:
            i = 0
            while i + 1 < len(data):
                byte1 = data[i]
                if (byte1 & 0xF0) == 0xA0:  # Gesture response
                    responses.append(("gesture", byte1, data[i+1]))
                    i += 2
                elif (byte1 & 0xF0) == 0xB0:  # Status response
                    responses.append(("status", byte1))
                    i += 1
                else:
                    i += 1
    except Exception:
        pass
    return responses


def main():
    parser = argparse.ArgumentParser(description="Stream DVS events to FPGA")
    parser.add_argument("--dvs", default="/dev/ttyACM0", 
                        help="DVS camera serial port (default: /dev/ttyACM0)")
    parser.add_argument("--fpga", default="/dev/ttyUSB1",
                        help="FPGA serial port (default: /dev/ttyUSB1)")
    parser.add_argument("--batch-size", type=int, default=100,
                        help="Events per batch (default: 100)")
    parser.add_argument("--batch-interval", type=float, default=0.01,
                        help="Batch interval in seconds (default: 0.01)")
    parser.add_argument("--debug", action="store_true",
                        help="Show debug info: raw bytes, decoded events, histograms")
    args = parser.parse_args()
    
    print(f"\n{'='*60}")
    print(f"Live EVT Stream to FPGA")
    print(f"{'='*60}")
    print(f"DVS Camera (GENX320):  {args.dvs}")
    print(f"FPGA (iCE40):          {args.fpga}")
    print(f"{'='*60}\n")
    
    # Open FPGA connection
    try:
        fpga_port = serial.Serial(args.fpga, 115200, timeout=0.1)
        print(f"[FPGA] Connected to {args.fpga}")
    except Exception as e:
        print(f"[FPGA] Failed to open {args.fpga}: {e}")
        sys.exit(1)
    
    # Test FPGA connection
    try:
        fpga_port.write(b'\xFF')  # Echo test
        time.sleep(0.1)
        response = fpga_port.read(1)
        if response == b'\x55':
            print(f"[FPGA] Echo test OK ✓")
        else:
            print(f"[FPGA] Echo test failed (got {response.hex()})")
    except Exception as e:
        print(f"[FPGA] Echo test error: {e}")
    
    # Start DVS reader thread
    decoder = EVTDecoder()
    stop_event = Event()
    dvs_thread = Thread(target=read_dvs_thread, args=(args.dvs, decoder, stop_event), daemon=True)
    dvs_thread.start()
    
    time.sleep(0.5)
    
    # Main event streaming loop
    print(f"\n[STREAM] Starting live stream... Press Ctrl+C to stop\n")
    print(f"{'Time':<8} {'Events':<8} {'Sent':<6} {'Gesture':<12} {'Status'}")
    print(f"{'-'*60}")
    
    start_time = time.time()
    total_events = 0
    total_sent = 0
    gesture_count = {}
    
    # For debug: track event locations
    x_histogram = {}
    y_histogram = {}
    
    try:
        while True:
            # Get available events from decoder
            events = decoder.get_events(args.batch_size)
            
            if args.debug and events:
                print(f"\n[DEBUG] Decoded {len(events)} events:")
                for i, (x, y, pol) in enumerate(events[:10]):  # Show first 10
                    print(f"  Event {i}: X={x:3d} Y={y:3d} POL={pol}")
                if len(events) > 10:
                    print(f"  ... and {len(events) - 10} more")
                
                # Update histograms
                for x, y, pol in events:
                    x_histogram[x] = x_histogram.get(x, 0) + 1
                    y_histogram[y] = y_histogram.get(y, 0) + 1
            
            if events:
                # Encode and send to FPGA, always print debug info for each event
                for x, y, pol in events:
                    packet = encode_event_packet(x, y, pol)
                    print(f"[SEND] X={x:3d} Y={y:3d} POL={pol} -> Packet: {packet.hex()}")
                    try:
                        fpga_port.write(packet)
                        total_sent += 1
                    except Exception as e:
                        print(f"[FPGA] Write error: {e}")
                        break
                total_events += len(events)
            
            # Check for gesture responses
            responses = read_fpga_responses(fpga_port)
            gesture_str = ""
            for resp in responses:
                if resp[0] == "gesture":
                    gesture = decode_gesture_response(resp[1], resp[2])
                    gesture_count[gesture] = gesture_count.get(gesture, 0) + 1
                    gesture_str = f"🎯 {gesture}"
            
            # Print status
            elapsed = time.time() - start_time
            if int(elapsed) % 1 == 0:  # Print every second
                status = "OK"
                if total_events > 0:
                    print(f"{elapsed:6.1f}s {total_events:<8} {total_sent:<6} {gesture_str:<12} {status}")
                
                # Show event distribution every 5 seconds
                if args.debug and int(elapsed) % 5 == 0 and x_histogram:
                    x_min, x_max = min(x_histogram.keys()), max(x_histogram.keys())
                    y_min, y_max = min(y_histogram.keys()), max(y_histogram.keys())
                    print(f"[DEBUG] Event location range: X=[{x_min}, {x_max}], Y=[{y_min}, {y_max}]")
            
            time.sleep(args.batch_interval)
            
    except KeyboardInterrupt:
        print(f"\n\n[STREAM] Stopping...\n")
    finally:
        stop_event.set()
        dvs_thread.join(timeout=1)
        fpga_port.close()
        
        print(f"{'='*60}")
        print(f"Stream Statistics")
        print(f"{'='*60}")
        print(f"Total events captured: {total_events}")
        print(f"Total events sent to FPGA: {total_sent}")
        print(f"Gestures recognized:")
        for gesture, count in sorted(gesture_count.items()):
            print(f"  {gesture}: {count}")
        print(f"{'='*60}")


if __name__ == "__main__":
    main()

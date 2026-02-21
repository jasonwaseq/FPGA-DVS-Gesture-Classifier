#!/usr/bin/env python3
"""Inspect raw EVT2 stream from GENX320 and test multiple bit-field layouts."""

import serial
import sys
import struct

def main():
    port_name = "/dev/ttyACM0"
    
    try:
        port = serial.Serial(port_name, 115200, timeout=1)
        print(f"Connected to {port_name}")
        print("Reading raw bytes for 5 seconds...\n")
        
        raw_data = bytearray()
        import time
        start = time.time()
        
        while time.time() - start < 5:
            data = port.read(1024)
            if data:
                raw_data.extend(data)
                sys.stdout.write(f"\rCaptured {len(raw_data)} bytes")
                sys.stdout.flush()
        
        port.close()
        
        print(f"\nTotal captured: {len(raw_data)} bytes\n")
        print("First 100 bytes (hex):")
        for i in range(0, min(100, len(raw_data)), 16):
            hex_str = " ".join(f"{b:02x}" for b in raw_data[i:i+16])
            print(f"  {i:04d}: {hex_str}")
        print("\n" + "="*70)
        print("Testing different EVT2 bit layouts:")
        print("="*70)
        n_words = len(raw_data) // 4
        print(f"Total 32-bit words: {n_words}")
        if n_words == 0:
            print("Not enough data!")
            return
        print("\nFirst 50 words decoded (trying standard EVT2 layout):")
        print("Word# | HEX        | Type | X    | Y    | POL |  Motion Direction")
        print("-" * 70)
        for i in range(min(50, n_words)):
            word_bytes = raw_data[i*4:(i+1)*4]
            word = int.from_bytes(word_bytes, 'little', signed=False)
            event_type = (word >> 28) & 0xF
            pol_a = (word >> 0) & 0x1
            x_a = (word >> 1) & 0x1FF
            y_a = (word >> 10) & 0x1FF
            y_b = (word >> 1) & 0x1FF
            x_b = (word >> 10) & 0x1FF
            y_c = (word >> 1) & 0x3FF
            x_c = (word >> 11) & 0x3FF
            if x_a <= 319 and y_a <= 319:
                x, y, pol = x_a, y_a, pol_a
                layout = "(A)"
            elif x_b <= 319 and y_b <= 319:
                x, y, pol = x_b, y_b, pol_a
                layout = "(B)"
            else:
                x, y, pol = x_c, y_c, pol_a
                layout = "(C)"
            x = min(x, 319)
            y = min(y, 319)
            motion = ""
            if 150 < y < 170:
                motion = "(Mid-Y)"
            elif y < 50:
                motion = "(Top)"
            elif y > 250:
                motion = "(Bottom)"
            elif x < 50:
                motion = "(Left)"
            elif x > 250:
                motion = "(Right)"
            print(f"{i:4d}  | {word:08x}   | {event_type:>2} | {x:3d}  | {y:3d}  | {pol} | {motion} {layout}")
        print("\n" + "="*70)
        print("Y-coordinate distribution (first 10000 events):")
        print("="*70)
        y_counts = {}
        for i in range(min(10000, n_words)):
            word_bytes = raw_data[i*4:(i+1)*4]
            word = int.from_bytes(word_bytes, 'little', signed=False)
            event_type = (word >> 28) & 0xF
            if event_type == 0x8:
                continue
            y = min((word >> 10) & 0x1FF, 319)
            y_counts[y] = y_counts.get(y, 0) + 1
        y_min, y_max = min(y_counts.keys()), max(y_counts.keys())
        print(f"Y range: {y_min} to {y_max}")
        print(f"Average Y: {sum(y*c for y,c in y_counts.items()) / sum(y_counts.values()):.1f}")
        for y in range(0, 320, 32):
            if y <= y_max:
                bin_count = sum(c for yy, c in y_counts.items() if y <= yy < y + 32)
                print(f"  Y[{y:3d}-{y+31:3d}]: {bin_count:6d} " + "█" * min(bin_count // 1000, 50))
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

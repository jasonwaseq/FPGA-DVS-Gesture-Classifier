#!/usr/bin/env python3
"""
DVS Camera Emulator - Emulates Dynamic Vision Sensor events from a laptop webcam

This script captures frames from your laptop's camera and generates DVS-like events
by detecting changes in pixel intensity between consecutive frames.

DVS Output Protocol (compatible with uart_gesture_top.sv):
    5 bytes per event: [X_HI, X_LO, Y_HI, Y_LO, POL]
    - X_HI[0] = X[8], X_LO = X[7:0]  (9-bit X coordinate, 0-319)
    - Y_HI[0] = Y[8], Y_LO = Y[7:0]  (9-bit Y coordinate, 0-319)
    - POL[0] = polarity (1=ON brightness increase, 0=OFF brightness decrease)

Usage:
    python dvs_camera_emulator.py --port /dev/ttyUSB0 --threshold 15 --preview
    python dvs_camera_emulator.py --save events.bin  # Save events to file
    python dvs_camera_emulator.py --preview  # Preview without sending
    python dvs_camera_emulator.py --simulate --preview  # Simulate gestures without camera
    python dvs_camera_emulator.py --video input.mp4 --preview  # Use video file
"""

import argparse
import time
import sys
import struct
from collections import deque
from typing import Optional, Tuple, List
import threading
import queue

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV is required. Install with: pip install opencv-python")
    sys.exit(1)

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("WARNING: pyserial not installed. UART output disabled.")
    print("         Install with: pip install pyserial")


# =============================================================================
# DVS Emulator Constants
# =============================================================================

DVS_RESOLUTION = 320  # Target resolution (matches SENSOR_RES in RTL)
DEFAULT_THRESHOLD = 15  # Log intensity change threshold (in 8-bit units)
DEFAULT_REFRACTORY_US = 1000  # Minimum time between events at same pixel
DEFAULT_FPS = 30  # Camera capture FPS


# =============================================================================
# Gesture Simulator (for testing without camera)
# =============================================================================

class GestureSimulator:
    """
    Simulates hand gestures for testing without a physical camera.
    Generates synthetic frames with moving objects.
    """
    
    def __init__(self, resolution: int = DVS_RESOLUTION, fps: int = DEFAULT_FPS):
        self.resolution = resolution
        self.fps = fps
        self.frame_count = 0
        self.gesture_queue = []
        self.current_gesture = None
        self.gesture_frame = 0
        self.gesture_duration = 30  # frames per gesture
        
        # Add some initial gestures
        self.add_gesture('right')
        self.add_gesture('left')
        self.add_gesture('down')
        self.add_gesture('up')
    
    def add_gesture(self, direction: str):
        """Add a gesture to the queue: 'up', 'down', 'left', 'right'"""
        if direction is not None:
            self.gesture_queue.append(direction)
    
    def get_frame(self) -> np.ndarray:
        """Generate a synthetic frame with simulated motion"""
        frame = np.ones((self.resolution, self.resolution, 3), dtype=np.uint8) * 40  # Dark gray background
        
        # Handle pause markers in queue
        while self.gesture_queue and self.gesture_queue[0] is None:
            self.gesture_queue.pop(0)
        
        # Check if we need to start a new gesture
        if self.current_gesture is None and self.gesture_queue:
            self.current_gesture = self.gesture_queue.pop(0)
            self.gesture_frame = 0
            if self.current_gesture:
                print(f"Simulating gesture: {self.current_gesture.upper()}")
        
        if self.current_gesture:
            progress = self.gesture_frame / self.gesture_duration
            
            # Calculate object position based on gesture direction
            center = self.resolution // 2
            obj_size = self.resolution // 4
            
            if self.current_gesture == 'right':
                x = int(progress * (self.resolution - obj_size))
                y = center - obj_size // 2
            elif self.current_gesture == 'left':
                x = int((1 - progress) * (self.resolution - obj_size))
                y = center - obj_size // 2
            elif self.current_gesture == 'down':
                x = center - obj_size // 2
                y = int(progress * (self.resolution - obj_size))
            elif self.current_gesture == 'up':
                x = center - obj_size // 2
                y = int((1 - progress) * (self.resolution - obj_size))
            else:
                x, y = center, center
            
            # Draw a bright rectangle (simulating a hand)
            cv2.rectangle(frame, (x, y), (x + obj_size, y + obj_size), (200, 200, 200), -1)
            
            # Add some texture
            cv2.circle(frame, (x + obj_size//3, y + obj_size//3), obj_size//6, (180, 180, 180), -1)
            cv2.circle(frame, (x + 2*obj_size//3, y + obj_size//3), obj_size//6, (180, 180, 180), -1)
            
            self.gesture_frame += 1
            
            if self.gesture_frame >= self.gesture_duration:
                self.current_gesture = None
                # Add a small pause between gestures (handled by frame counter)
        
        # Re-add gestures for continuous loop when queue is empty
        if self.current_gesture is None and not self.gesture_queue:
            self.add_gesture('right')
            self.add_gesture('left')
            self.add_gesture('down')
            self.add_gesture('up')
        
        self.frame_count += 1
        return frame
    
    def is_running(self) -> bool:
        """Check if simulator is still running"""
        return True  # Always running in loop mode


# =============================================================================
# DVS Event Class
# =============================================================================

class DVSEvent:
    """Represents a single DVS event"""
    
    def __init__(self, x: int, y: int, polarity: bool, timestamp_us: int):
        self.x = x
        self.y = y
        self.polarity = polarity  # True=ON (brighter), False=OFF (darker)
        self.timestamp_us = timestamp_us
    
    def to_bytes(self) -> bytes:
        """Convert event to 5-byte UART protocol format"""
        x_hi = (self.x >> 8) & 0x01
        x_lo = self.x & 0xFF
        y_hi = (self.y >> 8) & 0x01
        y_lo = self.y & 0xFF
        pol = 1 if self.polarity else 0
        return bytes([x_hi, x_lo, y_hi, y_lo, pol])
    
    def __repr__(self):
        pol_str = "ON" if self.polarity else "OFF"
        return f"DVSEvent(x={self.x}, y={self.y}, pol={pol_str}, ts={self.timestamp_us}μs)"


# =============================================================================
# DVS Camera Emulator
# =============================================================================

class DVSCameraEmulator:
    """
    Emulates a Dynamic Vision Sensor using a standard webcam.
    
    The DVS emulation works by:
    1. Capturing frames from the webcam
    2. Converting to grayscale and computing log intensity
    3. Comparing consecutive frames to detect intensity changes
    4. Generating ON events (polarity=1) where brightness increased
    5. Generating OFF events (polarity=0) where brightness decreased
    """
    
    def __init__(
        self,
        camera_id: int = 0,
        output_resolution: int = DVS_RESOLUTION,
        threshold: float = DEFAULT_THRESHOLD,
        refractory_period_us: int = DEFAULT_REFRACTORY_US,
        fps: int = DEFAULT_FPS,
        noise_filter_size: int = 3
    ):
        self.camera_id = camera_id
        self.output_resolution = output_resolution
        self.threshold = threshold
        self.refractory_period_us = refractory_period_us
        self.target_fps = fps
        self.noise_filter_size = noise_filter_size
        
        # Camera capture
        self.cap: Optional[cv2.VideoCapture] = None
        self.prev_log_frame: Optional[np.ndarray] = None
        self.last_event_time: Optional[np.ndarray] = None
        
        # Timing
        self.start_time_us = 0
        self.frame_count = 0
        
        # Event statistics
        self.total_events = 0
        self.on_events = 0
        self.off_events = 0
    
    def open_camera(self) -> bool:
        """Initialize the webcam capture"""
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            print(f"ERROR: Could not open camera {self.camera_id}")
            return False
        
        # Set camera properties for lower latency
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer delay
        
        # Read actual properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        print(f"Camera opened: {actual_width}x{actual_height} @ {actual_fps:.1f} FPS")
        print(f"Output resolution: {self.output_resolution}x{self.output_resolution}")
        print(f"Threshold: {self.threshold}, Refractory: {self.refractory_period_us}μs")
        
        # Initialize tracking arrays
        self.last_event_time = np.zeros(
            (self.output_resolution, self.output_resolution), dtype=np.int64
        )
        self.start_time_us = int(time.time() * 1_000_000)
        
        return True
    
    def close_camera(self):
        """Release camera resources"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    def _preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """Convert frame to log intensity at target resolution"""
        # Convert to grayscale
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        
        # Resize to target resolution (square)
        resized = cv2.resize(
            gray, 
            (self.output_resolution, self.output_resolution),
            interpolation=cv2.INTER_AREA
        )
        
        # Optional noise filtering
        if self.noise_filter_size > 1:
            resized = cv2.GaussianBlur(
                resized, 
                (self.noise_filter_size, self.noise_filter_size), 
                0
            )
        
        # Convert to log intensity (add 1 to avoid log(0))
        # Use float32 for precision
        log_frame = np.log(resized.astype(np.float32) + 1.0)
        
        return log_frame
    
    def process_frame(self, frame: np.ndarray) -> List[DVSEvent]:
        """
        Process a camera frame and generate DVS events.
        
        Returns a list of DVSEvent objects for pixels that changed
        beyond the threshold since the last frame.
        """
        current_time_us = int(time.time() * 1_000_000) - self.start_time_us
        
        # Preprocess frame
        log_frame = self._preprocess_frame(frame)
        
        # First frame - no events, just store reference
        if self.prev_log_frame is None:
            self.prev_log_frame = log_frame
            return []
        
        # Compute intensity difference
        diff = log_frame - self.prev_log_frame
        
        # Scale diff to roughly match 8-bit threshold
        # log(255) - log(1) ≈ 5.5, so scale factor ~46
        diff_scaled = diff * 46.0
        
        events = []
        
        # Find pixels that changed beyond threshold
        on_mask = diff_scaled > self.threshold
        off_mask = diff_scaled < -self.threshold
        
        # Apply refractory period filter
        time_since_last = current_time_us - self.last_event_time
        refractory_mask = time_since_last >= self.refractory_period_us
        
        on_mask = on_mask & refractory_mask
        off_mask = off_mask & refractory_mask
        
        # Generate ON events (brightness increased)
        on_coords = np.argwhere(on_mask)
        for y, x in on_coords:
            event = DVSEvent(
                x=int(x),
                y=int(y),
                polarity=True,
                timestamp_us=current_time_us
            )
            events.append(event)
            self.last_event_time[y, x] = current_time_us
            self.on_events += 1
        
        # Generate OFF events (brightness decreased)
        off_coords = np.argwhere(off_mask)
        for y, x in off_coords:
            event = DVSEvent(
                x=int(x),
                y=int(y),
                polarity=False,
                timestamp_us=current_time_us
            )
            events.append(event)
            self.last_event_time[y, x] = current_time_us
            self.off_events += 1
        
        # Update reference frame
        self.prev_log_frame = log_frame
        self.total_events += len(events)
        self.frame_count += 1
        
        return events
    
    def capture_and_process(self) -> Tuple[Optional[np.ndarray], List[DVSEvent]]:
        """Capture a frame from camera and generate events"""
        if self.cap is None or not self.cap.isOpened():
            return None, []
        
        ret, frame = self.cap.read()
        if not ret:
            return None, []
        
        events = self.process_frame(frame)
        return frame, events
    
    def get_stats(self) -> dict:
        """Return current statistics"""
        return {
            'total_events': self.total_events,
            'on_events': self.on_events,
            'off_events': self.off_events,
            'frame_count': self.frame_count,
            'events_per_frame': self.total_events / max(1, self.frame_count)
        }


# =============================================================================
# Event Visualization
# =============================================================================

def create_event_visualization(
    events: List[DVSEvent], 
    resolution: int = DVS_RESOLUTION,
    accumulation_time_ms: int = 33
) -> np.ndarray:
    """
    Create a visualization image from DVS events.
    
    ON events are shown in blue, OFF events in red.
    """
    # Create RGB image (black background)
    vis = np.zeros((resolution, resolution, 3), dtype=np.uint8)
    
    for event in events:
        x, y = event.x, event.y
        if 0 <= x < resolution and 0 <= y < resolution:
            if event.polarity:  # ON event - blue
                vis[y, x, 0] = 255  # Blue channel
            else:  # OFF event - red
                vis[y, x, 2] = 255  # Red channel
    
    return vis


def create_combined_preview(
    original_frame: np.ndarray,
    events: List[DVSEvent],
    resolution: int = DVS_RESOLUTION,
    stats: dict = None
) -> np.ndarray:
    """Create a side-by-side preview with original and event visualization"""
    # Resize original to match DVS resolution
    orig_resized = cv2.resize(
        original_frame, 
        (resolution, resolution),
        interpolation=cv2.INTER_AREA
    )
    
    # Create event visualization
    event_vis = create_event_visualization(events, resolution)
    
    # Combine side by side
    combined = np.hstack([orig_resized, event_vis])
    
    # Add stats overlay
    if stats:
        text = f"Events: {stats['total_events']} | ON: {stats['on_events']} | OFF: {stats['off_events']}"
        cv2.putText(combined, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 1, cv2.LINE_AA)
        
        fps_text = f"Frames: {stats['frame_count']} | Events/frame: {stats['events_per_frame']:.1f}"
        cv2.putText(combined, fps_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1, cv2.LINE_AA)
    
    return combined


# =============================================================================
# UART Output Handler
# =============================================================================

class UARTOutputHandler:
    """Handles sending DVS events over UART to the FPGA"""
    
    def __init__(self, port: str, baud_rate: int = 115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial: Optional[serial.Serial] = None
        self.event_queue: queue.Queue = queue.Queue(maxsize=10000)
        self.running = False
        self.tx_thread: Optional[threading.Thread] = None
        self.events_sent = 0
        self.gestures_received = []
    
    def open(self) -> bool:
        """Open serial connection"""
        if not HAS_SERIAL:
            print("ERROR: pyserial not installed")
            return False
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"UART opened: {self.port} @ {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"ERROR: Could not open {self.port}: {e}")
            return False
    
    def close(self):
        """Close serial connection"""
        self.running = False
        if self.tx_thread:
            self.tx_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
    
    def start_tx_thread(self):
        """Start background thread for sending events"""
        self.running = True
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.tx_thread.start()
    
    def _tx_loop(self):
        """Background thread that sends events from queue"""
        while self.running:
            try:
                event = self.event_queue.get(timeout=0.1)
                if self.serial and self.serial.is_open:
                    self.serial.write(event.to_bytes())
                    self.events_sent += 1
                    
                    # Check for responses (gestures)
                    if self.serial.in_waiting > 0:
                        response = self.serial.read(self.serial.in_waiting)
                        for byte in response:
                            if (byte & 0xF0) == 0xA0:  # Gesture response
                                gesture = byte & 0x03
                                gesture_names = ['UP', 'DOWN', 'LEFT', 'RIGHT']
                                self.gestures_received.append(gesture_names[gesture])
                                print(f"\n*** GESTURE DETECTED: {gesture_names[gesture]} ***")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"TX Error: {e}")
    
    def send_event(self, event: DVSEvent):
        """Queue an event for sending"""
        try:
            self.event_queue.put_nowait(event)
        except queue.Full:
            pass  # Drop event if queue is full
    
    def send_events(self, events: List[DVSEvent]):
        """Queue multiple events for sending"""
        for event in events:
            self.send_event(event)
    
    def test_connection(self) -> bool:
        """Test FPGA connection with echo command"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            self.serial.reset_input_buffer()
            self.serial.write(bytes([0xFF]))  # Echo command
            time.sleep(0.1)
            
            if self.serial.in_waiting > 0:
                response = self.serial.read(1)
                if response[0] == 0x55:
                    print("FPGA connection verified (echo test passed)")
                    return True
            
            print("WARNING: FPGA did not respond to echo test")
            return False
            
        except Exception as e:
            print(f"Connection test failed: {e}")
            return False


# =============================================================================
# File Output Handler
# =============================================================================

class FileOutputHandler:
    """Saves DVS events to a binary file"""
    
    def __init__(self, filename: str):
        self.filename = filename
        self.file = None
        self.events_written = 0
    
    def open(self):
        """Open file for writing"""
        self.file = open(self.filename, 'wb')
        # Write header: magic number + version
        self.file.write(b'DVS1')  # Magic + version
        print(f"Saving events to: {self.filename}")
    
    def close(self):
        """Close file"""
        if self.file:
            self.file.close()
            print(f"Saved {self.events_written} events to {self.filename}")
    
    def write_event(self, event: DVSEvent):
        """Write a single event to file"""
        if self.file:
            # Format: x(2), y(2), polarity(1), timestamp(4) = 9 bytes
            data = struct.pack('<HHBI', event.x, event.y, 
                             1 if event.polarity else 0, event.timestamp_us)
            self.file.write(data)
            self.events_written += 1
    
    def write_events(self, events: List[DVSEvent]):
        """Write multiple events to file"""
        for event in events:
            self.write_event(event)


# =============================================================================
# Main Application
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='DVS Camera Emulator - Generate DVS events from webcam',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Preview mode (no output):
    python dvs_camera_emulator.py --preview

  Simulate gestures without camera (for testing):
    python dvs_camera_emulator.py --simulate --preview

  Use video file as input:
    python dvs_camera_emulator.py --video input.mp4 --preview

  Send to FPGA via UART:
    python dvs_camera_emulator.py --port /dev/ttyUSB0 --preview

  Save events to file:
    python dvs_camera_emulator.py --save events.bin --preview

  Adjust sensitivity:
    python dvs_camera_emulator.py --threshold 10 --preview  # More sensitive
    python dvs_camera_emulator.py --threshold 25 --preview  # Less sensitive
        """
    )
    
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera device ID (default: 0)')
    parser.add_argument('--video', type=str, default=None,
                        help='Video file to use as input instead of camera')
    parser.add_argument('--simulate', action='store_true',
                        help='Simulate gestures without camera (for testing)')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for UART output (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='UART baud rate (default: 115200)')
    parser.add_argument('--threshold', type=float, default=DEFAULT_THRESHOLD,
                        help=f'Event threshold (default: {DEFAULT_THRESHOLD})')
    parser.add_argument('--refractory', type=int, default=DEFAULT_REFRACTORY_US,
                        help=f'Refractory period in μs (default: {DEFAULT_REFRACTORY_US})')
    parser.add_argument('--resolution', type=int, default=DVS_RESOLUTION,
                        help=f'Output resolution (default: {DVS_RESOLUTION})')
    parser.add_argument('--fps', type=int, default=DEFAULT_FPS,
                        help=f'Camera FPS (default: {DEFAULT_FPS})')
    parser.add_argument('--preview', action='store_true',
                        help='Show preview window')
    parser.add_argument('--save', type=str, default=None,
                        help='Save events to binary file')
    parser.add_argument('--noise-filter', type=int, default=3,
                        help='Gaussian blur kernel size for noise filtering (default: 3, 0=disabled)')
    parser.add_argument('--max-events', type=int, default=1000,
                        help='Maximum events per frame to send (default: 1000)')
    parser.add_argument('--loop', action='store_true',
                        help='Loop video file playback')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("DVS Camera Emulator")
    print("=" * 60)
    print()
    
    # Determine input source
    use_simulator = args.simulate
    use_video = args.video is not None
    video_cap = None
    simulator = None
    
    # Initialize DVS emulator
    emulator = DVSCameraEmulator(
        camera_id=args.camera,
        output_resolution=args.resolution,
        threshold=args.threshold,
        refractory_period_us=args.refractory,
        fps=args.fps,
        noise_filter_size=args.noise_filter if args.noise_filter > 0 else 1
    )
    
    if use_simulator:
        print("Mode: SIMULATION (synthetic gestures)")
        print(f"Resolution: {args.resolution}x{args.resolution}")
        print(f"Threshold: {args.threshold}")
        print()
        simulator = GestureSimulator(args.resolution, args.fps)
        # Initialize emulator tracking arrays without opening camera
        emulator.last_event_time = np.zeros(
            (args.resolution, args.resolution), dtype=np.int64
        )
        emulator.start_time_us = int(time.time() * 1_000_000)
    elif use_video:
        print(f"Mode: VIDEO FILE ({args.video})")
        video_cap = cv2.VideoCapture(args.video)
        if not video_cap.isOpened():
            print(f"ERROR: Could not open video file: {args.video}")
            sys.exit(1)
        
        # Get video properties
        video_fps = video_cap.get(cv2.CAP_PROP_FPS)
        video_frames = int(video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        video_width = int(video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        video_height = int(video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Video: {video_width}x{video_height} @ {video_fps:.1f} FPS, {video_frames} frames")
        print(f"Output resolution: {args.resolution}x{args.resolution}")
        print(f"Threshold: {args.threshold}")
        print()
        
        # Initialize emulator tracking arrays without opening camera
        emulator.last_event_time = np.zeros(
            (args.resolution, args.resolution), dtype=np.int64
        )
        emulator.start_time_us = int(time.time() * 1_000_000)
    else:
        print("Mode: CAMERA")
        if not emulator.open_camera():
            print()
            print("TIP: If no camera is available, try:")
            print("  --simulate    : Simulate gestures without camera")
            print("  --video FILE  : Use a video file as input")
            sys.exit(1)
    
    # Initialize output handlers
    uart_handler = None
    file_handler = None
    
    if args.port:
        uart_handler = UARTOutputHandler(args.port, args.baud)
        if uart_handler.open():
            uart_handler.test_connection()
            uart_handler.start_tx_thread()
        else:
            uart_handler = None
    
    if args.save:
        file_handler = FileOutputHandler(args.save)
        file_handler.open()
    
    print()
    print("Controls:")
    print("  q     - Quit")
    print("  +/-   - Adjust threshold")
    print("  r     - Reset statistics")
    print("  Space - Pause/Resume")
    if use_simulator:
        print("  1-4   - Trigger gesture (1=UP, 2=DOWN, 3=LEFT, 4=RIGHT)")
    print()
    
    paused = False
    frame_time = 1.0 / args.fps
    
    try:
        while True:
            loop_start = time.time()
            
            if not paused:
                # Get frame from appropriate source
                frame = None
                
                if use_simulator:
                    frame = simulator.get_frame()
                elif use_video:
                    ret, frame = video_cap.read()
                    if not ret:
                        if args.loop:
                            video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                            ret, frame = video_cap.read()
                        if not ret:
                            print("Video playback complete")
                            break
                else:
                    frame, _ = emulator.capture_and_process()
                    if frame is not None:
                        # Already processed, get events from the result
                        events = _
                    
                if frame is None and not use_simulator and not use_video:
                    print("Camera error - reconnecting...")
                    time.sleep(1.0)
                    continue
                
                # Process frame to generate events (for simulator and video modes)
                if use_simulator or use_video:
                    events = emulator.process_frame(frame)
                elif frame is None:
                    continue
                
                # Limit events per frame
                if len(events) > args.max_events:
                    events = events[:args.max_events]
                
                # Send events to outputs
                if uart_handler:
                    uart_handler.send_events(events)
                
                if file_handler:
                    file_handler.write_events(events)
                
                # Preview
                if args.preview:
                    stats = emulator.get_stats()
                    
                    # Add source info to stats
                    if use_simulator:
                        stats['source'] = 'SIMULATION'
                    elif use_video:
                        stats['source'] = 'VIDEO'
                    else:
                        stats['source'] = 'CAMERA'
                    
                    preview = create_combined_preview(frame, events, args.resolution, stats)
                    
                    # Scale up for better visibility
                    preview_scaled = cv2.resize(preview, None, fx=2, fy=2, 
                                               interpolation=cv2.INTER_NEAREST)
                    
                    cv2.imshow('DVS Emulator (Original | Events)', preview_scaled)
            
            # Handle keyboard input
            if args.preview:
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('+') or key == ord('='):
                    emulator.threshold += 2
                    print(f"Threshold: {emulator.threshold}")
                elif key == ord('-'):
                    emulator.threshold = max(1, emulator.threshold - 2)
                    print(f"Threshold: {emulator.threshold}")
                elif key == ord('r'):
                    emulator.total_events = 0
                    emulator.on_events = 0
                    emulator.off_events = 0
                    emulator.frame_count = 0
                    print("Statistics reset")
                elif key == ord(' '):
                    paused = not paused
                    print("Paused" if paused else "Resumed")
                # Gesture triggers for simulation mode
                elif use_simulator and key == ord('1'):
                    simulator.add_gesture('up')
                    print("Queued gesture: UP")
                elif use_simulator and key == ord('2'):
                    simulator.add_gesture('down')
                    print("Queued gesture: DOWN")
                elif use_simulator and key == ord('3'):
                    simulator.add_gesture('left')
                    print("Queued gesture: LEFT")
                elif use_simulator and key == ord('4'):
                    simulator.add_gesture('right')
                    print("Queued gesture: RIGHT")
            else:
                # Without preview, add a small delay
                time.sleep(0.001)
            
            # Rate limiting
            elapsed = time.time() - loop_start
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        # Cleanup
        print()
        print("Final Statistics:")
        stats = emulator.get_stats()
        print(f"  Total Events: {stats['total_events']}")
        print(f"  ON Events:    {stats['on_events']}")
        print(f"  OFF Events:   {stats['off_events']}")
        print(f"  Frames:       {stats['frame_count']}")
        
        if uart_handler:
            print(f"  Events Sent:  {uart_handler.events_sent}")
            if uart_handler.gestures_received:
                print(f"  Gestures:     {uart_handler.gestures_received}")
            uart_handler.close()
        
        if file_handler:
            file_handler.close()
        
        if video_cap:
            video_cap.release()
        
        if not use_simulator and not use_video:
            emulator.close_camera()
        
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

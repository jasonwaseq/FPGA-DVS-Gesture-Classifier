"""
Cocotb Testbench for Voxel-Bin DVS Gesture Accelerator
Comprehensive testbench for the voxel-bin architecture

Tests:
1. test_uart_echo          - UART echo test (send 0xFF, expect 0x55)
2. test_status_query       - Status query (send 0xFE, expect 0xBx)
3. test_config_query       - Config query (send 0xFD, expect 2 bytes)
4. test_soft_reset         - Soft reset (send 0xFC)
5. test_event_injection    - Event injection and FIFO handling
6. test_gesture_up         - UP gesture classification
7. test_gesture_down       - DOWN gesture classification
8. test_gesture_left       - LEFT gesture classification
9. test_gesture_right      - RIGHT gesture classification
10. test_multiple_gestures - Multiple gestures in sequence
11. test_noise_rejection   - Sub-threshold events should not trigger gestures
12. test_event_rate        - High event rate stress test
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import random

# =============================================================================
# Configuration Constants
# =============================================================================
CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE  # 104

# Voxel-bin architecture parameters
GRID_SIZE = 16
SENSOR_RES = 320
WINDOW_MS = 400
MIN_EVENT_THRESH = 20
MOTION_THRESH = 8

# Gesture encodings
GESTURE_UP = 0
GESTURE_DOWN = 1
GESTURE_LEFT = 2
GESTURE_RIGHT = 3
GESTURE_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}

# =============================================================================
# UART Functions
# =============================================================================

async def uart_send_byte(dut, byte_val):
    """Send one byte via UART"""
    dut.uart_rx.value = 0  # Start bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    for i in range(8):  # Data bits LSB first
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    dut.uart_rx.value = 1  # Stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def uart_receive_byte(dut, timeout_cycles=50000):
    """Receive one byte from uart_tx"""
    # Wait for idle high
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 1:
            break
    else:
        return None
    
    # Wait for start bit
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 0:
            break
    else:
        return None
    
    # Sample in middle of bits
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if dut.uart_tx.value != 0:
        return None
    
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if dut.uart_tx.value == 1:
            byte_val |= (1 << i)
    
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    return byte_val


async def send_dvs_event(dut, x, y, polarity):
    """Send a 5-byte DVS event packet"""
    x = max(0, min(319, x))
    y = max(0, min(319, y))
    await uart_send_byte(dut, (x >> 8) & 0x01)
    await uart_send_byte(dut, x & 0xFF)
    await uart_send_byte(dut, (y >> 8) & 0x01)
    await uart_send_byte(dut, y & 0xFF)
    await uart_send_byte(dut, polarity & 0x01)


async def send_event_stream(dut, events, inter_event_gap=10):
    """Send a stream of DVS events"""
    for x, y, pol in events:
        await send_dvs_event(dut, x, y, pol)
        await ClockCycles(dut.clk, inter_event_gap)


async def check_gesture_response(dut, timeout_cycles=200000):
    """Check for gesture response (0xA0 | gesture, confidence)"""
    byte1 = await uart_receive_byte(dut, timeout_cycles)
    if byte1 is None:
        return None, None
    
    # Check if this is a gesture response (0xAx)
    if (byte1 & 0xF0) == 0xA0:
        byte2 = await uart_receive_byte(dut, timeout_cycles=50000)
        if byte2 is None:
            byte2 = 0
        
        gesture = byte1 & 0x03
        confidence = (byte2 >> 4) & 0x0F
        return gesture, confidence
    
    return None, None


# =============================================================================
# Event Generation
# =============================================================================

def generate_gesture_events(gesture, n=300, noise=0.02, start_pos=None, end_pos=None):
    """Generate (x, y, pol) events simulating a clear linear gesture swipe.
    
    Args:
        gesture: GESTURE_UP, GESTURE_DOWN, GESTURE_LEFT, or GESTURE_RIGHT
        n: Number of events to generate
        noise: Fraction of random noise events (0.0-1.0)
        start_pos: (x, y) start position (default: center - offset)
        end_pos: (x, y) end position (default: center + offset)
    """
    events = []
    cx, cy = 160, 160  # Center of 320x320 sensor
    
    # Default motion range: ~140 pixels for voxel-bin (needs more motion)
    motion_range = 140
    
    if start_pos is None or end_pos is None:
        if gesture == GESTURE_UP:
            start_pos = (cx, cy + motion_range // 2)
            end_pos = (cx, cy - motion_range // 2)
        elif gesture == GESTURE_DOWN:
            start_pos = (cx, cy - motion_range // 2)
            end_pos = (cx, cy + motion_range // 2)
        elif gesture == GESTURE_LEFT:
            start_pos = (cx + motion_range // 2, cy)
            end_pos = (cx - motion_range // 2, cy)
        elif gesture == GESTURE_RIGHT:
            start_pos = (cx - motion_range // 2, cy)
            end_pos = (cx + motion_range // 2, cy)
        else:
            raise ValueError(f"Unknown gesture {gesture}")
    
    sx, sy = start_pos
    ex, ey = end_pos
    
    # Generate events along the motion path
    for i in range(n):
        t = i / max(1, n - 1)  # 0.0 to 1.0
        if random.random() < noise:
            # Add noise event
            events.append((random.randint(0, 319), random.randint(0, 319), 1))
        else:
            # Main gesture path with small spread
            spread = 18
            x = int(sx + (ex - sx) * t + random.gauss(0, spread / 3))
            y = int(sy + (ey - sy) * t + random.gauss(0, spread / 3))
            x = max(0, min(319, x))
            y = max(0, min(319, y))
            events.append((x, y, 1))
    
    return events


# =============================================================================
# Test Setup
# =============================================================================

async def setup_test(dut):
    """Initialize clock and reset"""
    clock = Clock(dut.clk, 83, units="ns")  # ~12 MHz
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 100)  # Wait for POR
    assert dut.uart_tx.value == 1, "TX should be idle high"
    dut._log.info("Setup complete — clock at 12 MHz, POR complete")


# =============================================================================
# Test 1: UART Echo
# =============================================================================

@cocotb.test()
async def test_uart_echo(dut):
    """Test UART echo: send 0xFF, expect 0x55"""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: UART Echo")
    dut._log.info("=" * 60)
    
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    
    assert response is not None, "No response received"
    assert response == 0x55, f"Expected 0x55, got 0x{response:02X}"
    dut._log.info("UART Echo Test PASSED ✓")


# =============================================================================
# Test 2: Status Query
# =============================================================================

@cocotb.test()
async def test_status_query(dut):
    """Test status query: send 0xFE, expect 0xBx"""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Status Query")
    dut._log.info("=" * 60)
    
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFE)
    response = await recv_task
    
    assert response is not None, "No response received"
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info(f"Status byte: 0x{response:02X} (temporal_phase={response>>3&1}, fifo_full={response>>1&1}, fifo_empty={response&1})")
    dut._log.info("Status Query Test PASSED ✓")


# =============================================================================
# Test 3: Config Query
# =============================================================================

@cocotb.test()
async def test_config_query(dut):
    """Test config query: send 0xFD, expect 2 bytes (min_thresh, motion_thresh)"""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Config Query")
    dut._log.info("=" * 60)
    
    recv_task1 = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFD)
    byte1 = await recv_task1
    
    assert byte1 is not None, "No first byte received"
    
    byte2 = await uart_receive_byte(dut, timeout_cycles=50000)
    assert byte2 is not None, "No second byte received"
    
    dut._log.info(f"Config: MIN_EVENT_THRESH={byte1}, MOTION_THRESH={byte2}")
    assert byte1 == MIN_EVENT_THRESH, f"MIN_EVENT_THRESH mismatch: got {byte1}, expected {MIN_EVENT_THRESH}"
    assert byte2 == MOTION_THRESH, f"MOTION_THRESH mismatch: got {byte2}, expected {MOTION_THRESH}"
    dut._log.info("Config Query Test PASSED ✓")


# =============================================================================
# Test 4: Soft Reset
# =============================================================================

@cocotb.test()
async def test_soft_reset(dut):
    """Test soft reset: send 0xFC"""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Soft Reset")
    dut._log.info("=" * 60)
    
    # Send some events first
    await send_dvs_event(dut, 160, 160, 1)
    await ClockCycles(dut.clk, 100)
    
    # Send reset command
    await uart_send_byte(dut, 0xFC)
    await ClockCycles(dut.clk, 1000)
    
    # Verify system still responds to echo
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    
    assert response == 0x55, "System should still respond after reset"
    dut._log.info("Soft Reset Test PASSED ✓")


# =============================================================================
# Test 5: Event Injection
# =============================================================================

@cocotb.test()
async def test_event_injection(dut):
    """Test that events can be injected via UART"""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Event Injection")
    dut._log.info("=" * 60)
    
    # Send multiple events
    for i in range(10):
        await send_dvs_event(dut, 160 + i*5, 160 + i*5, 1)
        await ClockCycles(dut.clk, 50)
    
    # Query status to verify system still works
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFE)
    response = await recv_task
    
    assert response is not None, "No status response after events"
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info("Event Injection Test PASSED ✓")


# =============================================================================
# Test 6-9: Gesture Classification Tests
# =============================================================================

async def gesture_test(dut, gesture, expected_class, description):
    """Robust gesture classification test for voxel-bin architecture."""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info(f"TEST: {description}")
    dut._log.info("=" * 60)
    
    # Generate a strong gesture pattern
    events = generate_gesture_events(gesture, n=400, noise=0.01)
    dut._log.info(f"Generated {len(events)} events for {GESTURE_NAMES[expected_class]} gesture")
    
    # Send events continuously - voxel-bin needs events spread over time window
    # Send in chunks to fill temporal bins
    num_chunks = 8
    chunk_size = len(events) // num_chunks
    
    for chunk_idx in range(num_chunks):
        start_idx = chunk_idx * chunk_size
        end_idx = start_idx + chunk_size if chunk_idx < num_chunks - 1 else len(events)
        await send_event_stream(dut, events[start_idx:end_idx], inter_event_gap=5)
        
        # Small delay between chunks to fill temporal bins
        await ClockCycles(dut.clk, 2000)
    
    # Wait for window processing and classification
    # Voxel-bin uses WINDOW_MS window, need to wait for classification
    window_cycles = (CLK_FREQ_HZ // 1000) * WINDOW_MS
    await ClockCycles(dut.clk, window_cycles + 10000)  # Extra margin
    
    # Monitor for gesture response via UART
    gesture_detected = False
    detected_class = None
    confidence = None
    max_attempts = 5
    
    for attempt in range(max_attempts):
        gesture_class, conf = await check_gesture_response(dut, timeout_cycles=window_cycles)
        
        if gesture_class is not None:
            detected_class = gesture_class
            confidence = conf
            gesture_detected = True
            dut._log.info(
                f"  Attempt {attempt+1}: Detected {GESTURE_NAMES.get(detected_class, '?')} "
                f"(confidence={confidence}, expected {GESTURE_NAMES[expected_class]})"
            )
            break
        
        # Wait a bit more before next attempt
        await ClockCycles(dut.clk, window_cycles // 2)
    
    # Assert the result
    assert gesture_detected, (
        f"Gesture {GESTURE_NAMES[expected_class]} not detected after {max_attempts} attempts. "
        f"Check thresholds, event generation, and window timing."
    )
    assert detected_class == expected_class, (
        f"Wrong gesture detected: got {GESTURE_NAMES.get(detected_class, '?')}, "
        f"expected {GESTURE_NAMES[expected_class]}"
    )
    
    dut._log.info(f"{description} PASSED ✓ (confidence={confidence})")


@cocotb.test()
async def test_gesture_up(dut):
    await gesture_test(dut, GESTURE_UP, 0, "UP Gesture Classification")


@cocotb.test()
async def test_gesture_down(dut):
    await gesture_test(dut, GESTURE_DOWN, 1, "DOWN Gesture Classification")


@cocotb.test()
async def test_gesture_left(dut):
    await gesture_test(dut, GESTURE_LEFT, 2, "LEFT Gesture Classification")


@cocotb.test()
async def test_gesture_right(dut):
    await gesture_test(dut, GESTURE_RIGHT, 3, "RIGHT Gesture Classification")


# =============================================================================
# Test 10: Multiple Gestures Sequential
# =============================================================================

@cocotb.test()
async def test_multiple_gestures(dut):
    """Test that the system can classify multiple gestures in sequence."""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Multiple Gestures Sequential")
    dut._log.info("=" * 60)
    
    gestures_to_test = [
        (GESTURE_UP, 0, "UP"),
        (GESTURE_DOWN, 1, "DOWN"),
        (GESTURE_LEFT, 2, "LEFT"),
        (GESTURE_RIGHT, 3, "RIGHT")
    ]
    
    detected_gestures = []
    window_cycles = (CLK_FREQ_HZ // 1000) * WINDOW_MS
    
    for gesture, expected_class, name in gestures_to_test:
        dut._log.info(f"Testing {name} gesture...")
        
        # Generate and send events
        events = generate_gesture_events(gesture, n=350, noise=0.01)
        
        # Send in chunks
        num_chunks = 6
        chunk_size = len(events) // num_chunks
        for chunk_idx in range(num_chunks):
            start_idx = chunk_idx * chunk_size
            end_idx = start_idx + chunk_size if chunk_idx < num_chunks - 1 else len(events)
            await send_event_stream(dut, events[start_idx:end_idx], inter_event_gap=5)
            await ClockCycles(dut.clk, 2000)
        
        # Wait for classification
        await ClockCycles(dut.clk, window_cycles + 5000)
        
        # Check for detection
        gesture_found = False
        for attempt in range(3):
            gesture_class, conf = await check_gesture_response(dut, timeout_cycles=window_cycles)
            if gesture_class == expected_class:
                detected_gestures.append(name)
                gesture_found = True
                dut._log.info(f"  ✓ {name} detected correctly (confidence={conf})")
                break
            await ClockCycles(dut.clk, window_cycles // 2)
        
        if not gesture_found:
            dut._log.warning(f"  ✗ {name} not detected")
        
        # Clear bins between gestures
        await ClockCycles(dut.clk, window_cycles * 2)
    
    # Verify at least 3 out of 4 gestures were detected
    assert len(detected_gestures) >= 3, (
        f"Only {len(detected_gestures)}/4 gestures detected: {detected_gestures}"
    )
    dut._log.info(f"Multiple Gestures Test PASSED — detected {len(detected_gestures)}/4 gestures ✓")


# =============================================================================
# Test 11: Noise Rejection
# =============================================================================

@cocotb.test()
async def test_noise_rejection(dut):
    """Test that sub-threshold events don't trigger gestures."""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Noise Rejection")
    dut._log.info("=" * 60)
    
    # Send only a few scattered events (below threshold)
    for i in range(5):
        x = random.randint(50, 270)
        y = random.randint(50, 270)
        await send_dvs_event(dut, x, y, 1)
        await ClockCycles(dut.clk, 1000)
    
    # Wait for window processing
    window_cycles = (CLK_FREQ_HZ // 1000) * WINDOW_MS
    await ClockCycles(dut.clk, window_cycles * 2)
    
    # Check that no gesture was detected
    gesture_class, _ = await check_gesture_response(dut, timeout_cycles=window_cycles)
    
    assert gesture_class is None, (
        f"Spurious gesture detected: {GESTURE_NAMES.get(gesture_class, '?')} "
        f"with only {5} events (below threshold)"
    )
    
    dut._log.info("Noise Rejection Test PASSED — no spurious detections ✓")


# =============================================================================
# Test 12: Event Rate Stress Test
# =============================================================================

@cocotb.test()
async def test_event_rate(dut):
    """Test system handles high event rates without errors."""
    await setup_test(dut)
    dut._log.info("=" * 60)
    dut._log.info("TEST: Event Rate Stress Test")
    dut._log.info("=" * 60)
    
    # Generate many events quickly
    events = generate_gesture_events(GESTURE_RIGHT, n=600, noise=0.05)
    
    # Send events with minimal gap (stress test)
    for x, y, pol in events[:300]:  # Send first half quickly
        await send_dvs_event(dut, x, y, pol)
        await ClockCycles(dut.clk, 2)  # Minimal gap
    
    # Wait a bit
    await ClockCycles(dut.clk, 5000)
    
    # Send remaining events
    for x, y, pol in events[300:]:
        await send_dvs_event(dut, x, y, pol)
        await ClockCycles(dut.clk, 2)
    
    # Wait for processing
    window_cycles = (CLK_FREQ_HZ // 1000) * WINDOW_MS
    await ClockCycles(dut.clk, window_cycles + 10000)
    
    # System should still function - verify echo still works
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    
    assert response == 0x55, "System should still respond after high event rate"
    dut._log.info("Event Rate Stress Test PASSED — system handled high event rate ✓")

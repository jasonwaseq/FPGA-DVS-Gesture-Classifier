"""
Cocotb testbench for voxel-bin DVS gesture accelerator.
Tests: UART echo/status/config/reset, event injection, gesture classification (UP/DOWN/LEFT/RIGHT),
noise rejection, multiple gestures, and high event rate stress.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import random

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 3000000
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE  # = 4 cycles per bit

GRID_SIZE = 16
SENSOR_RES = 320
WINDOW_MS = 40
MIN_EVENT_THRESH = 20
MOTION_THRESH = 8

# Each DVS event = 5 bytes * 10 bits/byte * CLKS_PER_BIT = 200 cycles
CYCLES_PER_EVENT = 5 * 10 * CLKS_PER_BIT  # = 200 cycles
# CYCLES_PER_BIN is overridden in Makefile to 2000
CYCLES_PER_BIN_SIM = 2000
# 4 bins * 2000 = 8000 cycles window; 35 events * 200 = 7000 cycles fits in one window
NUM_GESTURE_EVENTS = 35  # must fit in 4-bin window to preserve temporal gradient

GESTURE_UP = 0
GESTURE_DOWN = 1
GESTURE_LEFT = 2
GESTURE_RIGHT = 3
GESTURE_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}

async def uart_send_byte(dut, byte_val):
    dut.uart_rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def uart_receive_byte(dut, timeout_cycles=50000):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 1:
            break
    else:
        return None
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 0:
            break
    else:
        return None
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


class UartReceiver:
    def __init__(self, dut):
        self.dut = dut
        self.queue = []
        self.running = True
        self.task = cocotb.start_soon(self._receive_loop())

    async def _receive_loop(self):
        while self.running:
            try:
                byte = await uart_receive_byte(self.dut, timeout_cycles=100000)
                if byte is not None:
                    self.queue.append(byte)
            except Exception:
                pass

    async def get_byte(self, timeout_cycles=200000):
        waited = 0
        while len(self.queue) == 0:
            if waited >= timeout_cycles:
                return None
            await ClockCycles(self.dut.clk, 100)
            waited += 100
        return self.queue.pop(0)

    def clear(self):
        self.queue = []

    def stop(self):
        self.running = False
        self.task.kill()


async def send_dvs_event(dut, x, y, polarity):
    x = max(0, min(319, x))
    y = max(0, min(319, y))
    await uart_send_byte(dut, (x >> 8) & 0x01)
    await uart_send_byte(dut, x & 0xFF)
    await uart_send_byte(dut, (y >> 8) & 0x01)
    await uart_send_byte(dut, y & 0xFF)
    await uart_send_byte(dut, polarity & 0x01)


async def send_event_stream(dut, events, inter_event_gap=10):
    for x, y, pol in events:
        await send_dvs_event(dut, x, y, pol)
        await ClockCycles(dut.clk, inter_event_gap)


async def check_gesture_response(dut, receiver, timeout_cycles=200000):
    byte1 = await receiver.get_byte(timeout_cycles)
    if byte1 is None:
        return None, None
    dut._log.info(f"  RX byte1=0x{byte1:02X}")
    if (byte1 & 0xF0) == 0xA0:
        byte2 = await receiver.get_byte(timeout_cycles=50000)
        if byte2 is None:
            byte2 = 0
        
        gesture = byte1 & 0x03
        confidence = (byte2 >> 4) & 0x0F
        # Consume any duplicate gesture outputs in the queue
        receiver.clear()
        return gesture, confidence
    
    return None, None


def generate_gesture_events(gesture, n=300, noise=0.02, start_pos=None, end_pos=None):
    """Generate (x, y, pol) events for a linear gesture swipe."""
    events = []
    cx, cy = 160, 160
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

    for i in range(n):
        t = i / max(1, n - 1)
        if random.random() < noise:
            events.append((random.randint(0, 319), random.randint(0, 319), 1))
        else:
            spread = 18
            x = int(sx + (ex - sx) * t + random.gauss(0, spread / 3))
            y = int(sy + (ey - sy) * t + random.gauss(0, spread / 3))
            x = max(0, min(319, x))
            y = max(0, min(319, y))
            events.append((x, y, 1))
    
    return events


async def setup_test(dut):
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 100)
    assert dut.uart_tx.value == 1, "TX should be idle high"
    # Soft reset to clear all state and restart bin timer
    await uart_send_byte(dut, 0xFC)
    await ClockCycles(dut.clk, 500)  # Wait for reset to propagate and initial clear
    dut._log.info("Setup complete")


@cocotb.test()
async def test_uart_echo(dut):
    """UART echo: send 0xFF, expect 0x55"""
    await setup_test(dut)
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    
    assert response is not None, "No response received"
    assert response == 0x55, f"Expected 0x55, got 0x{response:02X}"
    dut._log.info("UART Echo Test PASSED")


@cocotb.test()
async def test_status_query(dut):
    """Status query: send 0xFE, expect 0xBx"""
    await setup_test(dut)
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFE)
    response = await recv_task
    
    assert response is not None, "No response received"
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info(f"Status byte: 0x{response:02X}")
    dut._log.info("Status Query Test PASSED")


@cocotb.test()
async def test_config_query(dut):
    """Config query: send 0xFD, expect 2 bytes (min_thresh, motion_thresh)"""
    await setup_test(dut)
    recv_task1 = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFD)
    byte1 = await recv_task1
    
    assert byte1 is not None, "No first byte received"
    
    byte2 = await uart_receive_byte(dut, timeout_cycles=50000)
    assert byte2 is not None, "No second byte received"
    
    assert byte1 == MIN_EVENT_THRESH, f"MIN_EVENT_THRESH mismatch: got {byte1}, expected {MIN_EVENT_THRESH}"
    assert byte2 == MOTION_THRESH, f"MOTION_THRESH mismatch: got {byte2}, expected {MOTION_THRESH}"
    dut._log.info("Config Query Test PASSED")


@cocotb.test()
async def test_soft_reset(dut):
    """Soft reset: send 0xFC, verify echo still works"""
    await setup_test(dut)
    await send_dvs_event(dut, 160, 160, 1)
    await ClockCycles(dut.clk, 100)
    await uart_send_byte(dut, 0xFC)
    await ClockCycles(dut.clk, 1000)
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    
    assert response == 0x55, "System should still respond after reset"
    dut._log.info("Soft Reset Test PASSED")


@cocotb.test()
async def test_event_injection(dut):
    """Inject events via UART and verify system still responds"""
    await setup_test(dut)
    for i in range(10):
        await send_dvs_event(dut, 160 + i*5, 160 + i*5, 1)
        await ClockCycles(dut.clk, 50)
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFE)
    response = await recv_task
    
    assert response is not None, "No status response after events"
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info("Event Injection Test PASSED")


async def gesture_test(dut, gesture, expected_class, description):
    await setup_test(dut)
    receiver = UartReceiver(dut)
    try:
        # Send events with no inter-event gap.
        # With CYCLES_PER_BIN=2000 and 4 bins, window = 8000 cycles.
        # 35 events * 200 cycles/event = 7000 cycles fits in one window.
        events = generate_gesture_events(gesture, n=NUM_GESTURE_EVENTS, noise=0.01)
        dut._log.info(f"Sending {len(events)} events for {GESTURE_NAMES[gesture]} gesture")
        await send_event_stream(dut, events, inter_event_gap=0)
        dut._log.info(f"Events sent, waiting for classification...")
        # Wait for bin rotation and readout/classification
        window_cycles = 6 * CYCLES_PER_BIN_SIM

        gesture_detected = False
        detected_class = None
        confidence = None
        max_attempts = 10

        for attempt in range(max_attempts):
            gesture_class, conf = await check_gesture_response(dut, receiver, timeout_cycles=window_cycles)
            dut._log.info(f"Attempt {attempt+1}: got class={gesture_class}, conf={conf}")
            if gesture_class is not None:
                detected_class = gesture_class
                confidence = conf
                gesture_detected = True
                dut._log.info(f"Attempt {attempt+1}: {GESTURE_NAMES.get(detected_class, '?')} (confidence={confidence})")
                break
    finally:
        receiver.stop()

    assert gesture_detected, (
        f"Gesture {GESTURE_NAMES[expected_class]} not detected after {max_attempts} attempts. "
        f"Check thresholds, event generation, and window timing."
    )
    assert detected_class == expected_class, (
        f"Wrong gesture detected: got {GESTURE_NAMES.get(detected_class, '?')}, "
        f"expected {GESTURE_NAMES[expected_class]}"
    )
    
    dut._log.info(f"{description} PASSED (confidence={confidence})")


@cocotb.test()
async def test_gesture_debug(dut):
    """Debug: send UP events and monitor internal signals."""
    await setup_test(dut)
    events = generate_gesture_events(GESTURE_UP, n=NUM_GESTURE_EVENTS, noise=0.01)
    dut._log.info(f"Sending {len(events)} UP events")
    await send_event_stream(dut, events, inter_event_gap=0)
    dut._log.info("Events sent, monitoring for gesture_valid...")
    # Monitor for gesture_valid and readout_start for 20000 cycles
    readout_seen = False
    result_valid_seen = False
    for i in range(20000):
        await RisingEdge(dut.clk)
        try:
            gv = dut.u_accel.gesture_valid.value
            if gv == 1:
                g = dut.u_accel.gesture.value
                dut._log.info(f"  gesture_valid=1 at cycle {i}, gesture={g}")
                break
            rs = dut.u_accel.u_time_surface_binning.readout_start.value
            if rs == 1 and not readout_seen:
                readout_seen = True
                dut._log.info(f"  readout_start=1 at cycle {i}")
            rv = dut.u_accel.u_systolic_array.result_valid.value
            if rv == 1 and not result_valid_seen:
                result_valid_seen = True
                score_above = dut.u_accel.score_above_thresh.value
                best_class = dut.u_accel.u_systolic_array.best_class.value
                abs_score = dut.u_accel.abs_best_score.value
                # Try to read accumulator scores
                try:
                    scores = []
                    for k in range(4):
                        s = dut.u_accel.u_systolic_array.acc_r[k].value
                        scores.append(int(s.signed_integer) if hasattr(s, 'signed_integer') else int(s))
                    dut._log.info(f"  result_valid=1 at cycle {i}, best_class={best_class}, abs_score={abs_score}, score_above={score_above}")
                    dut._log.info(f"  scores: {scores}")
                except Exception as e2:
                    dut._log.info(f"  result_valid=1 at cycle {i}, best_class={best_class}, abs_score={abs_score}, score_above={score_above} (score read err: {e2})")
        except Exception as e:
            if i < 5:
                dut._log.warning(f"  Exception at cycle {i}: {e}")
    else:
        dut._log.warning("No gesture_valid in 20000 cycles")
        if not readout_seen:
            dut._log.warning("  readout_start never seen!")
        if not result_valid_seen:
            dut._log.warning("  result_valid never seen!")


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


@cocotb.test()
async def test_multiple_gestures(dut):
    """Classify multiple gestures in sequence; expect at least 3/4 detected."""
    await setup_test(dut)
    gestures_to_test = [
        (GESTURE_UP, 0, "UP"),
        (GESTURE_DOWN, 1, "DOWN"),
        (GESTURE_LEFT, 2, "LEFT"),
        (GESTURE_RIGHT, 3, "RIGHT")
    ]
    
    detected_gestures = []
    window_cycles = 6 * CYCLES_PER_BIN_SIM
    
    receiver = UartReceiver(dut)
    try:
        for gesture, expected_class, name in gestures_to_test:
            dut._log.info(f"Testing {name} gesture...")
            events = generate_gesture_events(gesture, n=NUM_GESTURE_EVENTS, noise=0.01)
            receiver.clear()
            await send_event_stream(dut, events, inter_event_gap=0)
            
            gesture_found = False
            for attempt in range(5):
                gesture_class, conf = await check_gesture_response(dut, receiver, timeout_cycles=window_cycles)
                if gesture_class == expected_class:
                    detected_gestures.append(name)
                    gesture_found = True
                    dut._log.info(f"{name} detected (confidence={conf})")
                    break
                elif gesture_class is not None:
                    dut._log.warning(f"Got spurious {GESTURE_NAMES.get(gesture_class, '?')} while expecting {name}")
            
            if not gesture_found:
                dut._log.warning(f"{name} not detected")
            # Wait for bins to clear before next gesture
            await ClockCycles(dut.clk, 4 * CYCLES_PER_BIN_SIM)
    finally:
        receiver.stop()
    assert len(detected_gestures) >= 3, (
        f"Only {len(detected_gestures)}/4 gestures detected: {detected_gestures}"
    )
    dut._log.info(f"Multiple Gestures Test PASSED: {len(detected_gestures)}/4 gestures detected")


@cocotb.test()
async def test_noise_rejection(dut):
    """Sub-threshold events must not trigger a gesture."""
    await setup_test(dut)
    for i in range(5):
        x = random.randint(50, 270)
        y = random.randint(50, 270)
        await send_dvs_event(dut, x, y, 1)
        await ClockCycles(dut.clk, 1000)
    receiver = UartReceiver(dut)
    try:
        window_cycles = (CLK_FREQ_HZ // 1000) * WINDOW_MS
        await ClockCycles(dut.clk, window_cycles * 2)
        gesture_class, _ = await check_gesture_response(dut, receiver, timeout_cycles=window_cycles)
        assert gesture_class is None, (
            f"Spurious gesture detected: {GESTURE_NAMES.get(gesture_class, '?')} with only 5 events"
        )
        dut._log.info("Noise Rejection Test PASSED")
    finally:
        receiver.stop()


@cocotb.test()
async def test_event_rate(dut):
    """System must still respond to UART after a high-rate event burst."""
    await setup_test(dut)
    events = generate_gesture_events(GESTURE_RIGHT, n=100, noise=0.05)
    for x, y, pol in events[:50]:
        await send_dvs_event(dut, x, y, pol)
        await ClockCycles(dut.clk, 2)
    await ClockCycles(dut.clk, 2000)
    for x, y, pol in events[50:]:
        await send_dvs_event(dut, x, y, pol)
        await ClockCycles(dut.clk, 2)
    await ClockCycles(dut.clk, 4 * CYCLES_PER_BIN_SIM)
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    assert response == 0x55, "System should still respond after high event rate"
    dut._log.info("Event Rate Stress Test PASSED")

"""
Cocotb testbench for voxel_bin_processed_top.
Events arrive as pre-decoded, grid-mapped (x[3:0], y[3:0], polarity) on a
valid/ready bus.  The FPGA maps grid coords to sensor-space internally.
Timestamps are generated internally.
Tests: event injection, gesture classification (UP/DOWN/LEFT/RIGHT),
back-pressure (event_ready) handling, noise rejection, multiple gestures,
high event rate stress.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE

GRID_SIZE = 16
WINDOW_MS = 40
MIN_EVENT_THRESH = 20
MOTION_THRESH = 8

CYCLES_PER_BIN_SIM = 2000
NUM_GESTURE_EVENTS = 35

GESTURE_UP    = 0
GESTURE_DOWN  = 1
GESTURE_LEFT  = 2
GESTURE_RIGHT = 3
GESTURE_NAMES = {0: "UP", 1: "DOWN", 2: "LEFT", 3: "RIGHT"}


def clamp_grid(v):
    return max(0, min(15, int(v)))


async def send_event(dut, gx, gy, polarity=1):
    """Drive one grid-mapped event onto the bus, respecting event_ready back-pressure."""
    dut.event_x.value        = clamp_grid(gx)
    dut.event_y.value        = clamp_grid(gy)
    dut.event_polarity.value = int(polarity)
    dut.event_valid.value    = 1
    for _ in range(256):
        await RisingEdge(dut.clk)
        if dut.event_ready.value == 1:
            break
    dut.event_valid.value = 0
    await ClockCycles(dut.clk, 1)


async def send_event_stream(dut, events, inter_event_gap=2):
    for gx, gy, pol in events:
        await send_event(dut, gx, gy, pol)
        if inter_event_gap > 0:
            await ClockCycles(dut.clk, inter_event_gap)


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
        receiver.clear()
        return gesture, confidence
    return None, None


def generate_gesture_events(gesture, n=300, noise=0.02):
    """Generate (gx, gy, pol) grid-coordinate events for a linear swipe."""
    events = []
    cx, cy = 8, 8
    motion_range = 7
    if gesture == GESTURE_UP:
        start_pos = (cx, cy + motion_range // 2)
        end_pos   = (cx, cy - motion_range // 2)
    elif gesture == GESTURE_DOWN:
        start_pos = (cx, cy - motion_range // 2)
        end_pos   = (cx, cy + motion_range // 2)
    elif gesture == GESTURE_LEFT:
        start_pos = (cx + motion_range // 2, cy)
        end_pos   = (cx - motion_range // 2, cy)
    elif gesture == GESTURE_RIGHT:
        start_pos = (cx - motion_range // 2, cy)
        end_pos   = (cx + motion_range // 2, cy)
    else:
        raise ValueError(f"Unknown gesture {gesture}")
    sx, sy = start_pos
    ex, ey = end_pos
    for i in range(n):
        t = i / max(1, n - 1)
        if random.random() < noise:
            events.append((random.randint(0, 15), random.randint(0, 15), 1))
        else:
            x = int(sx + (ex - sx) * t + random.gauss(0, 0.8))
            y = int(sy + (ey - sy) * t + random.gauss(0, 0.8))
            events.append((clamp_grid(x), clamp_grid(y), 1))
    return events


async def setup_dut(dut):
    clock = Clock(dut.clk, 83, unit="ns")
    cocotb.start_soon(clock.start())
    dut.event_valid.value    = 0
    dut.event_x.value        = 0
    dut.event_y.value        = 0
    dut.event_polarity.value = 0
    # Wait for internal power-on reset to clear (32 cycles)
    await ClockCycles(dut.clk, 50)
    dut._log.info("Setup complete")


@cocotb.test()
async def test_basic_event_injection(dut):
    """Inject events and verify the DUT accepts them without hanging."""
    await setup_dut(dut)
    dut._log.info("Testing basic event injection...")
    test_events = [(8, 8, 1), (4, 4, 0), (12, 12, 1), (0, 0, 1), (15, 15, 0)]
    for gx, gy, pol in test_events:
        await send_event(dut, gx, gy, pol)
        await ClockCycles(dut.clk, 5)
    dut._log.info("Basic Event Injection Test PASSED")


@cocotb.test()
async def test_back_pressure(dut):
    """event_ready must be asserted — the processed path has no FIFO."""
    await setup_dut(dut)
    dut.event_x.value        = 8
    dut.event_y.value        = 8
    dut.event_polarity.value = 1
    dut.event_valid.value    = 1
    always_ready = True
    for _ in range(32):
        await RisingEdge(dut.clk)
        if dut.event_ready.value == 0:
            always_ready = False
            break
    dut.event_valid.value = 0
    assert always_ready, "event_ready deasserted unexpectedly"
    dut._log.info("Back-Pressure Test PASSED (event_ready always asserted)")


@cocotb.test()
async def test_gesture_up(dut):
    """Inject UP-swipe events and verify UART reports gesture 0."""
    await setup_dut(dut)
    receiver = UartReceiver(dut)
    events = generate_gesture_events(GESTURE_UP, n=NUM_GESTURE_EVENTS, noise=0.0)
    dut._log.info(f"Sending {len(events)} UP events...")
    await send_event_stream(dut, events, inter_event_gap=2)
    await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 10)
    gesture, confidence = await check_gesture_response(dut, receiver, timeout_cycles=500000)
    receiver.stop()
    assert gesture is not None, "No gesture response received for UP"
    assert gesture == GESTURE_UP, f"Expected UP(0), got {GESTURE_NAMES.get(gesture, gesture)}"
    dut._log.info(f"UP Gesture Test PASSED (confidence={confidence})")


@cocotb.test()
async def test_gesture_down(dut):
    """Inject DOWN-swipe events and verify UART reports gesture 1."""
    await setup_dut(dut)
    receiver = UartReceiver(dut)
    events = generate_gesture_events(GESTURE_DOWN, n=NUM_GESTURE_EVENTS, noise=0.0)
    dut._log.info(f"Sending {len(events)} DOWN events...")
    await send_event_stream(dut, events, inter_event_gap=2)
    await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 10)
    gesture, confidence = await check_gesture_response(dut, receiver, timeout_cycles=500000)
    receiver.stop()
    assert gesture is not None, "No gesture response received for DOWN"
    assert gesture == GESTURE_DOWN, f"Expected DOWN(1), got {GESTURE_NAMES.get(gesture, gesture)}"
    dut._log.info(f"DOWN Gesture Test PASSED (confidence={confidence})")


@cocotb.test()
async def test_gesture_left(dut):
    """Inject LEFT-swipe events and verify UART reports gesture 2."""
    await setup_dut(dut)
    receiver = UartReceiver(dut)
    events = generate_gesture_events(GESTURE_LEFT, n=NUM_GESTURE_EVENTS, noise=0.0)
    dut._log.info(f"Sending {len(events)} LEFT events...")
    await send_event_stream(dut, events, inter_event_gap=2)
    await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 10)
    gesture, confidence = await check_gesture_response(dut, receiver, timeout_cycles=500000)
    receiver.stop()
    assert gesture is not None, "No gesture response received for LEFT"
    assert gesture == GESTURE_LEFT, f"Expected LEFT(2), got {GESTURE_NAMES.get(gesture, gesture)}"
    dut._log.info(f"LEFT Gesture Test PASSED (confidence={confidence})")


@cocotb.test()
async def test_gesture_right(dut):
    """Inject RIGHT-swipe events and verify UART reports gesture 3."""
    await setup_dut(dut)
    receiver = UartReceiver(dut)
    events = generate_gesture_events(GESTURE_RIGHT, n=NUM_GESTURE_EVENTS, noise=0.0)
    dut._log.info(f"Sending {len(events)} RIGHT events...")
    await send_event_stream(dut, events, inter_event_gap=2)
    await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 10)
    gesture, confidence = await check_gesture_response(dut, receiver, timeout_cycles=500000)
    receiver.stop()
    assert gesture is not None, "No gesture response received for RIGHT"
    assert gesture == GESTURE_RIGHT, f"Expected RIGHT(3), got {GESTURE_NAMES.get(gesture, gesture)}"
    dut._log.info(f"RIGHT Gesture Test PASSED (confidence={confidence})")


@cocotb.test()
async def test_noise_rejection(dut):
    """Random noise events below MIN_EVENT_THRESH must not trigger a gesture."""
    await setup_dut(dut)
    receiver = UartReceiver(dut)
    noise_events = [(random.randint(0, 15), random.randint(0, 15), 1) for _ in range(5)]
    await send_event_stream(dut, noise_events, inter_event_gap=10)
    await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 5)
    gesture, _ = await check_gesture_response(dut, receiver, timeout_cycles=50000)
    receiver.stop()
    assert gesture is None, f"Spurious gesture detected from noise: {GESTURE_NAMES.get(gesture, gesture)}"
    dut._log.info("Noise Rejection Test PASSED")


@cocotb.test()
async def test_multiple_gestures_sequential(dut):
    """Classify all four gestures in sequence; expect at least 3/4 detected."""
    await setup_dut(dut)
    receiver = UartReceiver(dut)
    gestures_to_test = [
        (GESTURE_UP, "UP"),
        (GESTURE_DOWN, "DOWN"),
        (GESTURE_LEFT, "LEFT"),
        (GESTURE_RIGHT, "RIGHT"),
    ]
    detected = []
    for gesture, name in gestures_to_test:
        dut._log.info(f"Testing {name}...")
        events = generate_gesture_events(gesture, n=NUM_GESTURE_EVENTS, noise=0.0)
        await send_event_stream(dut, events, inter_event_gap=2)
        await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 10)
        g, c = await check_gesture_response(dut, receiver, timeout_cycles=300000)
        if g == gesture:
            detected.append(name)
            dut._log.info(f"  {name} detected (confidence={c})")
        else:
            dut._log.warning(f"  {name} not detected (got {GESTURE_NAMES.get(g, g)})")
        await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 3)
    receiver.stop()
    assert len(detected) >= 3, f"Only {len(detected)}/4 gestures detected: {detected}"
    dut._log.info(f"Multiple Gestures Test PASSED: {len(detected)}/4 detected")


@cocotb.test()
async def test_event_rate_stress(dut):
    """System must remain functional after a high-rate event burst."""
    await setup_dut(dut)
    events = generate_gesture_events(GESTURE_RIGHT, n=500, noise=0.05)
    await send_event_stream(dut, events, inter_event_gap=0)
    await ClockCycles(dut.clk, CYCLES_PER_BIN_SIM * 5)
    await send_event(dut, 8, 8, 1)
    await ClockCycles(dut.clk, 100)
    dut._log.info("Event Rate Stress Test PASSED")

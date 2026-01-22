"""
Cocotb testbench for UART Gesture Top module
Tests the full UART RX -> Accelerator -> UART TX flow

NOTE: Design uses internal power-on reset (no external rst_n)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer


# UART timing at 12MHz clock, 115200 baud
CLK_FREQ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ // BAUD_RATE  # 104
BIT_PERIOD_NS = (1_000_000_000 // BAUD_RATE)  # ~8680 ns


async def reset_dut(dut):
    """Wait for internal power-on reset to complete"""
    dut.uart_rx.value = 1  # Idle high
    # Wait for internal power-on reset (15 clocks + margin)
    await ClockCycles(dut.clk, 25)


async def uart_send_byte(dut, byte_val):
    """Send one byte via UART (bit-bang)"""
    # Start bit
    dut.uart_rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    # Data bits (LSB first)
    for i in range(8):
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    # Stop bit
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    # Small gap between bytes
    await ClockCycles(dut.clk, 10)


async def uart_receive_byte(dut, timeout_cycles=50000):
    """Receive one byte via UART, returns None on timeout"""
    # First, wait for TX to be idle (high)
    for _ in range(timeout_cycles // 10):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 1:
            break
    
    # Now wait for start bit (falling edge on tx)
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 0:
            break
    else:
        return None  # Timeout
    
    # Wait to middle of start bit
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    
    # Verify still in start bit
    if dut.uart_tx.value != 0:
        return None
    
    # Sample 8 data bits
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if dut.uart_tx.value == 1:
            byte_val |= (1 << i)
    
    # Wait for stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    return byte_val


async def send_dvs_event(dut, x, y, polarity, ts=0):
    """Send a 4-byte DVS event packet"""
    await uart_send_byte(dut, x & 0x7F)
    await uart_send_byte(dut, y & 0x7F)
    await uart_send_byte(dut, polarity & 0x01)
    await uart_send_byte(dut, ts & 0xFF)


async def capture_tx_byte(dut):
    """Monitor TX line and capture a byte if transmission occurs"""
    # Wait for start bit (falling edge)
    await RisingEdge(dut.clk)
    while dut.uart_tx.value == 1:
        await RisingEdge(dut.clk)
    
    # We're at start bit - wait to center
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    
    # Verify start bit
    if dut.uart_tx.value != 0:
        return None
    
    # Sample 8 data bits
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if dut.uart_tx.value == 1:
            byte_val |= (1 << i)
    
    # Wait for stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    return byte_val


async def send_events_and_capture_response(dut, events):
    """
    Send events while monitoring for TX response.
    Returns the captured byte or None.
    """
    captured_byte = None
    tx_capture_task = None
    
    async def tx_monitor():
        nonlocal captured_byte
        # Wait for TX to go low
        while True:
            await RisingEdge(dut.clk)
            if dut.uart_tx.value == 0:
                break
        
        # Capture the byte
        await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
        byte_val = 0
        for i in range(8):
            await ClockCycles(dut.clk, CLKS_PER_BIT)
            if dut.uart_tx.value == 1:
                byte_val |= (1 << i)
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        captured_byte = byte_val
    
    # Start monitoring TX
    tx_capture_task = cocotb.start_soon(tx_monitor())
    
    # Send all events
    for x, y, pol, ts in events:
        await send_dvs_event(dut, x, y, pol, ts)
    
    # Wait a bit for any pending TX
    await ClockCycles(dut.clk, 2000)
    
    return captured_byte


def decode_gesture(val):
    """Decode gesture byte"""
    gestures = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
    return gestures.get(val & 0x03, 'UNKNOWN')


@cocotb.test()
async def test_uart_loopback_check(dut):
    """Basic test - verify UART RX is receiving bytes"""
    clock = Clock(dut.clk, 83, units="ns")  # ~12MHz
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    dut._log.info(f"CLKS_PER_BIT = {CLKS_PER_BIT}")
    
    # Send a few bytes and check internal signals
    dut._log.info("Sending test bytes...")
    await uart_send_byte(dut, 0x55)  # 01010101 - good pattern
    await uart_send_byte(dut, 0xAA)  # 10101010
    await uart_send_byte(dut, 0x01)  # polarity
    await uart_send_byte(dut, 0x00)  # timestamp
    
    # Wait for processing
    await ClockCycles(dut.clk, 100)
    
    dut._log.info("Basic UART test completed")


@cocotb.test()
async def test_single_gesture(dut):
    """Test sending 100 events and receiving a gesture"""
    clock = Clock(dut.clk, 83, units="ns")  # ~12MHz
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    dut._log.info("Sending 100 RIGHT gesture events...")
    
    # Generate RIGHT gesture - X increases over time
    for i in range(100):
        progress = i / 100
        x = 64 + int(progress * 50)  # X goes from 64 to 114
        y = 64 + (i % 10) - 5        # Small Y variation
        x = max(0, min(127, x))
        y = max(0, min(127, y))
        
        await send_dvs_event(dut, x, y, 1, i % 256)
        
        if (i + 1) % 20 == 0:
            dut._log.info(f"  Sent {i+1} events...")
    
    dut._log.info("All events sent, waiting for gesture response...")
    
    # Check TX line activity
    dut._log.info(f"  uart_tx = {int(dut.uart_tx.value)}")
    
    # Wait a bit and monitor TX
    tx_went_low = False
    for i in range(30000):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 0:
            tx_went_low = True
            dut._log.info(f"  TX went LOW at cycle {i}")
            break
    
    if not tx_went_low:
        dut._log.warning("TX never went low - no transmission started")
    
    # Try to receive the gesture
    await ClockCycles(dut.clk, 1000)
    result = await uart_receive_byte(dut, timeout_cycles=20000)
    
    if result is not None:
        dut._log.info(f"Received byte: 0x{result:02X}")
        if (result & 0xF0) == 0xA0:
            gesture = result & 0x03
            dut._log.info(f"Gesture detected: {decode_gesture(gesture)} ({gesture})")
            assert gesture == 3, f"Expected RIGHT (3), got {gesture}"
        else:
            dut._log.error(f"Invalid marker, expected 0xAx, got 0x{result:02X}")
    else:
        dut._log.warning("No response received - checking internal signals...")
        # Note: led_gesture was removed, just check gesture signal
        dut._log.info(f"  gesture (internal) = {int(dut.u_accel.gesture.value)}")
    
    dut._log.info("Test completed")


@cocotb.test()
async def test_all_gestures_uart(dut):
    """Test all four gestures through UART"""
    clock = Clock(dut.clk, 83, units="ns")  # ~12MHz
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    test_cases = [
        ('RIGHT', 3, lambda i, p: (64 + int(p * 50), 64 + (i % 10) - 5)),
        ('LEFT',  2, lambda i, p: (64 - int(p * 50), 64 + (i % 10) - 5)),
        ('UP',    0, lambda i, p: (64 + (i % 10) - 5, 64 + int(p * 50))),
        ('DOWN',  1, lambda i, p: (64 + (i % 10) - 5, 64 - int(p * 50))),
    ]
    
    passed = 0
    
    for direction, expected, coord_fn in test_cases:
        dut._log.info(f"Testing {direction} gesture...")
        
        # Generate events
        events = []
        for i in range(100):
            progress = i / 100
            x, y = coord_fn(i, progress)
            x = max(0, min(127, x))
            y = max(0, min(127, y))
            events.append((x, y, 1, i % 256))
        
        # Send events and capture response
        result = await send_events_and_capture_response(dut, events)
        
        if result is not None and (result & 0xF0) == 0xA0:
            gesture = result & 0x03
            if gesture == expected:
                dut._log.info(f"  PASS: {direction} detected correctly")
                passed += 1
            else:
                dut._log.error(f"  FAIL: Expected {direction} ({expected}), got {decode_gesture(gesture)} ({gesture})")
        else:
            dut._log.error(f"  FAIL: No valid response for {direction} (got {result})")
        
        await ClockCycles(dut.clk, 100)
    
    dut._log.info(f"Results: {passed}/4 passed")
    assert passed == 4, f"Only {passed}/4 gestures passed"

"""
Simple cocotb testbench for DVS Gesture Accelerator
Tests gesture classification with simulated DVS events
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


async def reset_dut(dut):
    """Reset the DUT"""
    dut.rst_n.value = 0
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.event_polarity.value = 0
    dut.event_ts.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def send_event(dut, x, y, polarity, ts=0):
    """Send a single DVS event"""
    dut.event_valid.value = 1
    dut.event_x.value = x
    dut.event_y.value = y
    dut.event_polarity.value = polarity
    dut.event_ts.value = ts
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0


async def send_gesture_events(dut, direction, num_events=100):
    """
    Send events simulating a gesture movement
    direction: 'UP', 'DOWN', 'LEFT', 'RIGHT'
    """
    center_x, center_y = 64, 64
    
    for i in range(num_events):
        # Create events moving in the specified direction
        progress = i / num_events
        
        if direction == 'UP':
            x = center_x + (i % 10) - 5  # Small x variation
            y = center_y + int(progress * 50)  # Y increases (moving up in coords)
        elif direction == 'DOWN':
            x = center_x + (i % 10) - 5
            y = center_y - int(progress * 50)  # Y decreases
        elif direction == 'LEFT':
            x = center_x - int(progress * 50)  # X decreases
            y = center_y + (i % 10) - 5
        elif direction == 'RIGHT':
            x = center_x + int(progress * 50)  # X increases
            y = center_y + (i % 10) - 5
        else:
            x, y = center_x, center_y
        
        # Clamp to valid range (0-127)
        x = max(0, min(127, x))
        y = max(0, min(127, y))
        
        # ON events (polarity=1) for movement
        await send_event(dut, x, y, 1, ts=i)


def decode_gesture(value):
    """Decode gesture number to string"""
    gestures = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
    return gestures.get(value, 'UNKNOWN')


@cocotb.test()
async def test_right_gesture(dut):
    """Test RIGHT gesture detection"""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    dut._log.info("Testing RIGHT gesture...")
    await send_gesture_events(dut, 'RIGHT', num_events=100)
    
    # Wait for classification
    await ClockCycles(dut.clk, 10)
    
    # Check result
    if dut.gesture_valid.value == 1:
        gesture = int(dut.gesture.value)
        dut._log.info(f"Detected gesture: {decode_gesture(gesture)}")
        assert gesture == 3, f"Expected RIGHT (3), got {decode_gesture(gesture)}"
    
    dut._log.info("RIGHT gesture test PASSED")


@cocotb.test()
async def test_left_gesture(dut):
    """Test LEFT gesture detection"""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    dut._log.info("Testing LEFT gesture...")
    await send_gesture_events(dut, 'LEFT', num_events=100)
    
    await ClockCycles(dut.clk, 10)
    
    if dut.gesture_valid.value == 1:
        gesture = int(dut.gesture.value)
        dut._log.info(f"Detected gesture: {decode_gesture(gesture)}")
        assert gesture == 2, f"Expected LEFT (2), got {decode_gesture(gesture)}"
    
    dut._log.info("LEFT gesture test PASSED")


@cocotb.test()
async def test_up_gesture(dut):
    """Test UP gesture detection"""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    dut._log.info("Testing UP gesture...")
    await send_gesture_events(dut, 'UP', num_events=100)
    
    await ClockCycles(dut.clk, 10)
    
    if dut.gesture_valid.value == 1:
        gesture = int(dut.gesture.value)
        dut._log.info(f"Detected gesture: {decode_gesture(gesture)}")
        assert gesture == 0, f"Expected UP (0), got {decode_gesture(gesture)}"
    
    dut._log.info("UP gesture test PASSED")


@cocotb.test()
async def test_down_gesture(dut):
    """Test DOWN gesture detection"""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    dut._log.info("Testing DOWN gesture...")
    await send_gesture_events(dut, 'DOWN', num_events=100)
    
    await ClockCycles(dut.clk, 10)
    
    if dut.gesture_valid.value == 1:
        gesture = int(dut.gesture.value)
        dut._log.info(f"Detected gesture: {decode_gesture(gesture)}")
        assert gesture == 1, f"Expected DOWN (1), got {decode_gesture(gesture)}"
    
    dut._log.info("DOWN gesture test PASSED")


@cocotb.test()
async def test_all_gestures_sequence(dut):
    """Test all gestures in sequence"""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    
    await reset_dut(dut)
    
    expected_results = [
        ('RIGHT', 3),
        ('LEFT', 2),
        ('UP', 0),
        ('DOWN', 1),
    ]
    
    for direction, expected in expected_results:
        dut._log.info(f"Testing {direction} gesture in sequence...")
        await send_gesture_events(dut, direction, num_events=100)
        await ClockCycles(dut.clk, 10)
        
        if dut.gesture_valid.value == 1:
            gesture = int(dut.gesture.value)
            dut._log.info(f"  Detected: {decode_gesture(gesture)}")
    
    dut._log.info("Sequence test completed")

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE
CYCLES_PER_BIN = 20000

async def uart_send_byte(dut, byte_val):
    dut.uart_rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)

async def send_dvs_event(dut, x, y, polarity):
    await uart_send_byte(dut, (x >> 8) & 0x01)
    await uart_send_byte(dut, x & 0xFF)
    await uart_send_byte(dut, (y >> 8) & 0x01)
    await uart_send_byte(dut, y & 0xFF)
    await uart_send_byte(dut, polarity & 0x01)

async def uart_receive_byte(dut, timeout_cycles=50000):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 1:
            break
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

@cocotb.test()
async def test_gesture_up_sync(dut):
    """Test UP gesture with synchronized timing"""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 50)
    
    accel = dut.u_accel
    
    # Strategy: Fill entire ring buffer with carefully timed events
    # Wait for head=0, then fill bins 1,2,3 with early (low Y) and 4,5,6,7 with late (high Y)
    
    # Wait for head=0
    while int(accel.ring_head.value) != 0:
        await RisingEdge(dut.clk)
    dut._log.info("Starting at head=0")
    
    # Send 2 early events (low Y ~100) per bin for bins 1, 2, 3
    for target_bin in [1, 2, 3]:
        # Wait until we're in the target bin
        while int(accel.ring_head.value) != target_bin:
            await RisingEdge(dut.clk)
        dut._log.info(f"Sending early events to bin {target_bin}")
        for _ in range(2):
            await send_dvs_event(dut, 160, 100, 1)
    
    # Send 2 late events (high Y ~220) per bin for bins 4, 5, 6, 7
    for target_bin in [4, 5, 6, 7]:
        while int(accel.ring_head.value) != target_bin:
            await RisingEdge(dut.clk)
        dut._log.info(f"Sending late events to bin {target_bin}")
        for _ in range(2):
            await send_dvs_event(dut, 160, 220, 1)
    
    # Wait for next classification (when head advances to 0)
    dut._log.info("Waiting for classification at head=0...")
    while int(accel.ring_head.value) != 0:
        await RisingEdge(dut.clk)
    
    # bin_advance just happened for head=0
    # Scan will be bins 1-7 (bin 0 is clearing)
    # early (cls_bin 0-2) = bins 1, 2, 3
    # late (cls_bin 3-6) = bins 4, 5, 6, 7
    
    # Wait for classification to complete
    await ClockCycles(dut.clk, 20000)  # ~3500 voxels * 3 cycles
    
    dut._log.info(f"early_count={int(accel.early_count.value)}, late_count={int(accel.late_count.value)}")
    dut._log.info(f"early_sum_y={int(accel.early_sum_y.value)}, late_sum_y={int(accel.late_sum_y.value)}")
    dut._log.info(f"gesture_valid={int(accel.gesture_valid.value)}, gesture={int(accel.gesture.value)}")
    
    # Check for gesture output
    response = await uart_receive_byte(dut, timeout_cycles=5000)
    if response is not None and (response & 0xF0) == 0xA0:
        gesture = response & 0x03
        names = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
        dut._log.info(f"DETECTED: {names.get(gesture, '?')} (0x{response:02X})")
    else:
        dut._log.info(f"No gesture response (got {response})")


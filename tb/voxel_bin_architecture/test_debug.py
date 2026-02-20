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

@cocotb.test()
async def test_debug_gesture(dut):
    """Debug gesture detection"""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 50)
    
    accel = dut.u_accel
    
    # Send 3 early events (low Y) into bin 0
    dut._log.info("Sending 3 early events to bin 0...")
    for i in range(3):
        await send_dvs_event(dut, 160, 100, 1)
        dut._log.info(f"  event {i}: bin={int(accel.ring_head.value)}, evt_state={int(accel.evt_state.value)}")
    
    # Wait a bit, check voxel memory directly
    await ClockCycles(dut.clk, 100)
    
    # Map y=100 -> grid_y = (100*13)>>8 = 5
    # Map x=160 -> grid_x = (160*13)>>8 = 8
    # Address = {bin=0, y=5, x=8, pol=1} = 0*512 + 5*32 + 8*2 + 1 = 177
    dut._log.info(f"Expected grid: x=8, y=5, pol=1")
    dut._log.info(f"Check evt_addr after events: 0x{int(accel.evt_addr.value):03X}")
    
    # Wait for 5 bins to advance (ensure we're past early bins)
    dut._log.info("Waiting for 5 bins to pass...")
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 5)
    dut._log.info(f"Now at bin={int(accel.ring_head.value)}")
    
    # Send 3 late events (high Y) into current bin
    dut._log.info("Sending 3 late events...")
    for i in range(3):
        await send_dvs_event(dut, 160, 220, 1)
        dut._log.info(f"  event {i}: bin={int(accel.ring_head.value)}")
    
    # Now trigger classification by waiting for next bin_advance
    dut._log.info("Waiting for classification...")
    prev_bin = int(accel.ring_head.value)
    for _ in range(CYCLES_PER_BIN * 3):
        await RisingEdge(dut.clk)
        if int(accel.bin_advance.value) == 1:
            dut._log.info(f"bin_advance! head={int(accel.ring_head.value)}, cls_state={int(accel.cls_state.value)}")
            break
    
    # Wait for classification to complete (scan ~3500 voxels × 3 cycles)
    for _ in range(50000):
        await RisingEdge(dut.clk)
        if int(accel.cls_state.value) == 0:  # Back to IDLE
            break
        if int(accel.gesture_valid.value) == 1:
            dut._log.info(f"GESTURE VALID! gesture={int(accel.gesture.value)}")
            break
    
    dut._log.info(f"Final: cls_state={int(accel.cls_state.value)}")
    dut._log.info(f"early_count={int(accel.early_count.value)}, late_count={int(accel.late_count.value)}")
    dut._log.info(f"early_sum_y={int(accel.early_sum_y.value)}, late_sum_y={int(accel.late_sum_y.value)}")


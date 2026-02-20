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
async def test_debug_scan(dut):
    """Debug classification scan"""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 50)
    
    accel = dut.u_accel
    
    # Send 3 early events to bin 0 at position (160, 100)
    dut._log.info("Sending 3 events to bin 0, y=100...")
    for i in range(3):
        await send_dvs_event(dut, 160, 100, 1)
    
    await ClockCycles(dut.clk, 100)
    dut._log.info(f"Event addr used: 0x{int(accel.evt_addr.value):03X}")
    
    # Wait for classification to start
    for _ in range(CYCLES_PER_BIN * 2):
        await RisingEdge(dut.clk)
        if int(accel.bin_advance.value) == 1:
            dut._log.info(f"bin_advance! head={int(accel.ring_head.value)}, snapshot={int(accel.cls_snapshot_head.value)}")
            break
    
    # Monitor first few scan iterations of cls_bin=0, looking for our address
    # Our events are at address 0x0B1 = {bin=0, y=5, x=8, pol=1}
    # When scanning bin 0: cls_addr should hit 0x0B1 at cls_x=8, cls_y=5, cls_pol=1
    target_addr = 0x0B1
    
    for cycle in range(15000):  # Scan first bin: 16*16*2 = 512 cycles * 3 states
        await RisingEdge(dut.clk)
        state = int(accel.cls_state.value)
        if state == 0:  # IDLE
            break
        cls_bin = int(accel.cls_bin.value)
        if cls_bin > 0:
            break  # Only check first bin
        
        cls_addr = int(accel.cls_addr.value)
        if cls_addr == target_addr:
            counter = int(accel.cls_counter.value)
            dut._log.info(f"Found target addr! cls_addr=0x{cls_addr:03X}, counter={counter}")
            dut._log.info(f"  cls_bin={cls_bin}, cls_x={int(accel.cls_x.value)}, cls_y={int(accel.cls_y.value)}, cls_pol={int(accel.cls_pol.value)}")
            dut._log.info(f"  is_early={int(accel.is_early_bin.value)}")
            
    dut._log.info(f"After bin 0 scan: early_count={int(accel.early_count.value)}")


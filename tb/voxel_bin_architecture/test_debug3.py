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
async def test_scan_addresses(dut):
    """Debug what addresses classification scans"""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 50)
    
    accel = dut.u_accel
    
    # Send 3 events to bin 0 at y=100 (grid_y=5), x=160 (grid_x=8)
    dut._log.info("Sending events to bin 0...")
    for _ in range(3):
        await send_dvs_event(dut, 160, 100, 1)
    
    # Event address = {bin=0, y=5, x=8, pol=1} = 0*512 + 5*32 + 8*2 + 1 = 177 = 0x0B1
    dut._log.info(f"Events written to addr 0x0B1")
    
    # Wait for bin_advance
    for _ in range(CYCLES_PER_BIN * 2):
        await RisingEdge(dut.clk)
        if int(accel.bin_advance.value) == 1:
            break
    
    # Wait one more cycle for classification to start
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    
    dut._log.info(f"cls_snapshot_head = {int(accel.cls_snapshot_head.value)}")
    
    # Watch the first 10 addresses scanned
    dut._log.info("First 10 scan addresses:")
    for i in range(30):
        await RisingEdge(dut.clk)
        state = int(accel.cls_state.value)
        if state == 1:  # READ_SETUP - address is being computed
            addr = int(accel.cls_addr.value)
            cls_bin = int(accel.cls_bin.value)
            cls_x = int(accel.cls_x.value)
            cls_y = int(accel.cls_y.value)
            cls_pol = int(accel.cls_pol.value)
            if i < 30:
                # Decode address: {bin[2:0], y[3:0], x[3:0], pol}
                a_bin = (addr >> 9) & 0x7
                a_y = (addr >> 5) & 0xF
                a_x = (addr >> 1) & 0xF
                a_pol = addr & 0x1
                dut._log.info(f"  addr=0x{addr:03X} -> bin={a_bin}, y={a_y}, x={a_x}, pol={a_pol} (cls_bin={cls_bin})")


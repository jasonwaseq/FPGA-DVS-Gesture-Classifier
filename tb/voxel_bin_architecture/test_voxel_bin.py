"""
Cocotb Testbench for Voxel-Bin DVS Gesture Accelerator
Main testbench for the voxel-bin architecture

Tests:
- UART echo (send 0xFF, expect 0x55)
- Status query (send 0xFE, expect 0xBx)
- Event injection (verify events are accepted)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# ============================================================================
# Configuration - Fast simulation
# ============================================================================

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE  # 104

# ============================================================================
# UART Functions
# ============================================================================

async def uart_send_byte(dut, byte_val):
    """Send one byte via UART"""
    dut.uart_rx.value = 0  # Start bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    for i in range(8):  # Data bits LSB first
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    dut.uart_rx.value = 1  # Stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def uart_receive_byte(dut, timeout_cycles=20000):
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
    await uart_send_byte(dut, (x >> 8) & 0x01)
    await uart_send_byte(dut, x & 0xFF)
    await uart_send_byte(dut, (y >> 8) & 0x01)
    await uart_send_byte(dut, y & 0xFF)
    await uart_send_byte(dut, polarity & 0x01)


async def setup_test(dut):
    """Initialize clock and reset"""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 50)
    assert dut.uart_tx.value == 1, "TX should be idle high"


# ============================================================================
# Tests
# ============================================================================

@cocotb.test()
async def test_uart_echo(dut):
    """Test UART echo: send 0xFF, expect 0x55"""
    await setup_test(dut)
    dut._log.info("=== UART Echo Test ===")
    
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFF)
    response = await recv_task
    
    assert response is not None, "No response"
    assert response == 0x55, f"Expected 0x55, got 0x{response:02X}"
    dut._log.info("PASSED")


@cocotb.test()
async def test_status_query(dut):
    """Test status query: send 0xFE, expect 0xBx"""
    await setup_test(dut)
    dut._log.info("=== Status Query Test ===")
    
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFE)
    response = await recv_task
    
    assert response is not None, "No response"
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info(f"Current bin: {response & 0x07}")
    dut._log.info("PASSED")


@cocotb.test()
async def test_event_injection(dut):
    """Test that events can be injected via UART"""
    await setup_test(dut)
    dut._log.info("=== Event Injection Test ===")
    
    # Send a few events
    for i in range(3):
        await send_dvs_event(dut, 160 + i*10, 160 + i*10, 1)
        dut._log.info(f"Sent event {i+1}")
    
    # Query status to verify system still works
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    await uart_send_byte(dut, 0xFE)
    response = await recv_task
    
    assert response is not None, "No status response after events"
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info("PASSED")

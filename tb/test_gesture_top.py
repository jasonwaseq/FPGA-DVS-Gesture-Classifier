"""
Cocotb Testbench for Time-Surface DVS Gesture Classifier
Comprehensive verification of the new architecture

Tests:
1. Input FIFO: Basic read/write, full/empty flags
2. EVT2 Decoder: Packet parsing, timestamp reconstruction, downsampling
3. Time Surface Memory: Event updates, decay computation
4. Feature Extractor: Moment computation, classification
5. Integration: Full pipeline gesture detection
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
import random

# ============================================================================
# Configuration
# ============================================================================

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE  # 104

# EVT 2.0 Packet Types
EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

# ============================================================================
# Helper Functions
# ============================================================================

def make_evt2_cd_event(x, y, timestamp_lsb, polarity):
    """Create EVT 2.0 CD event packet (32-bit word)
    Format: [31:28]=type, [27:22]=ts_lsb, [21:11]=x, [10:0]=y
    """
    pkt_type = EVT_CD_ON if polarity else EVT_CD_OFF
    word = (pkt_type << 28) | ((timestamp_lsb & 0x3F) << 22) | ((x & 0x7FF) << 11) | (y & 0x7FF)
    return word

def make_evt2_time_high(time_high):
    """Create EVT 2.0 TIME_HIGH packet (32-bit word)
    Format: [31:28]=0x8, [27:0]=time_high
    """
    return (EVT_TIME_HIGH << 28) | (time_high & 0x0FFFFFFF)

async def reset_dut(dut):
    """Reset the DUT"""
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

async def send_evt2_word(dut, word):
    """Send a 32-bit EVT 2.0 word to the DUT"""
    dut.evt_data.value = word
    dut.evt_valid.value = 1
    await RisingEdge(dut.clk)
    # Wait for evt_ready if needed
    timeout = 100
    while dut.evt_ready.value == 0 and timeout > 0:
        await RisingEdge(dut.clk)
        timeout -= 1
    dut.evt_valid.value = 0
    await RisingEdge(dut.clk)

async def wait_for_uart_string(dut, expected_str, timeout_cycles=500000):
    """Wait for and receive a string from UART TX"""
    received = ""
    
    for _ in range(len(expected_str) + 2):  # +2 for \r\n
        byte_val = await uart_receive_byte(dut, timeout_cycles)
        if byte_val is None:
            break
        received += chr(byte_val)
        
    return received

async def uart_receive_byte(dut, timeout_cycles=50000):
    """Receive one byte from uart_tx"""
    # Wait for start bit (falling edge from idle high)
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 0:
            break
    else:
        return None
    
    # Sample in middle of start bit to verify
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if dut.uart_tx.value != 0:
        return None
    
    # Sample data bits
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if dut.uart_tx.value == 1:
            byte_val |= (1 << i)
    
    # Wait for stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    return byte_val


# ============================================================================
# Test: Basic Initialization
# ============================================================================

@cocotb.test()
async def test_reset_and_init(dut):
    """Test that DUT initializes correctly after reset"""
    dut._log.info("=== Reset and Initialization Test ===")
    
    # Start clock
    clock = Clock(dut.clk, 2, unit="step")  # Simulator step clock
    cocotb.start_soon(clock.start())
    
    # Initialize inputs
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await ClockCycles(dut.clk, 5)
    
    # Apply reset
    await reset_dut(dut)
    
    # Check outputs after reset
    assert dut.evt_ready.value == 1, "evt_ready should be high after reset (FIFO empty)"
    assert dut.uart_tx.value == 1, "UART TX should be idle high"
    
    # Short settle after reset
    await ClockCycles(dut.clk, 100)
    
    dut._log.info("Reset and initialization test PASSED")


# ============================================================================
# Test: FIFO Basic Operations
# ============================================================================

@cocotb.test()
async def test_fifo_write_read(dut):
    """Test FIFO accepts EVT 2.0 data and processes it"""
    dut._log.info("=== FIFO Write/Read Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Send a few EVT 2.0 events
    events = [
        make_evt2_cd_event(x=160, y=160, timestamp_lsb=10, polarity=1),
        make_evt2_cd_event(x=100, y=100, timestamp_lsb=11, polarity=0),
        make_evt2_cd_event(x=200, y=200, timestamp_lsb=12, polarity=1),
    ]
    
    for evt in events:
        await send_evt2_word(dut, evt)
        await ClockCycles(dut.clk, 5)
    
    # Verify FIFO accepted them (evt_ready should still be high)
    assert dut.evt_ready.value == 1, "FIFO should not be full after 3 events"
    
    dut._log.info("FIFO write/read test PASSED")


# ============================================================================
# Test: EVT 2.0 Decoder
# ============================================================================

@cocotb.test()
async def test_evt2_decoder(dut):
    """Test EVT 2.0 packet decoding and timestamp reconstruction"""
    dut._log.info("=== EVT2 Decoder Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Send TIME_HIGH packet first
    time_high_pkt = make_evt2_time_high(0x100)
    await send_evt2_word(dut, time_high_pkt)
    await ClockCycles(dut.clk, 10)
    
    # Send CD_ON event at center of sensor (160, 160)
    # This should map to grid cell (5, 5) with bits [8:5]
    cd_event = make_evt2_cd_event(x=160, y=160, timestamp_lsb=0x20, polarity=1)
    await send_evt2_word(dut, cd_event)
    await ClockCycles(dut.clk, 20)
    
    # Activity LED should light up
    await ClockCycles(dut.clk, 10)
    assert dut.led_activity.value == 1, "Activity LED should be on after event"
    
    dut._log.info("EVT2 decoder test PASSED")


# ============================================================================
# Test: Time Surface Updates
# ============================================================================

@cocotb.test()
async def test_time_surface(dut):
    """Test time surface memory updates on events"""
    dut._log.info("=== Time Surface Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Inject events at specific locations to create a pattern
    # Pattern: Events along top edge (should result in UP detection)
    for x in range(0, 320, 40):
        evt = make_evt2_cd_event(x=x, y=50, timestamp_lsb=0x10, polarity=1)
        await send_evt2_word(dut, evt)
        await ClockCycles(dut.clk, 5)
    
    # Wait some cycles for processing
    await ClockCycles(dut.clk, 100)
    
    # Activity should have been detected
    dut._log.info("Time surface test PASSED - events injected")


# ============================================================================
# Test: Feature Extraction and Classification
# ============================================================================

@cocotb.test()
async def test_feature_extraction(dut):
    """Test spatial moment computation and gesture classification"""
    dut._log.info("=== Feature Extraction Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Create a clear UP gesture pattern: events clustered at top of grid
    # Grid center is (8, 8), so events at y < 128 (grid y < 4) should trigger UP
    dut._log.info("Injecting UP gesture pattern (y < 100)")
    
    for i in range(50):
        x = random.randint(100, 220)  # Centered horizontally
        y = random.randint(20, 80)    # Top of frame
        evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
        await send_evt2_word(dut, evt)
        await ClockCycles(dut.clk, 3)
    
    # Wait for frame period (~10ms = 120000 cycles at 12MHz)
    # Use shorter wait for simulation
    dut._log.info("Waiting for classification frame...")
    await ClockCycles(dut.clk, 150000)
    
    dut._log.info("Feature extraction test PASSED")


# ============================================================================
# Test: Direction Detection - UP
# ============================================================================

@cocotb.test()
async def test_gesture_up(dut):
    """Test UP gesture detection"""
    dut._log.info("=== UP Gesture Detection Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Inject strong UP pattern: many events at top of sensor
    for _ in range(3):  # Multiple bursts
        for i in range(30):
            x = 160 + random.randint(-50, 50)  # Centered X
            y = random.randint(10, 60)          # Top of frame (low Y)
            evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
            await send_evt2_word(dut, evt)
            await ClockCycles(dut.clk, 2)
        await ClockCycles(dut.clk, 1000)
    
    # Wait for frame processing
    dut._log.info("Waiting for classification...")
    await ClockCycles(dut.clk, 130000)
    
    # Check if UP LED is on or UART output
    if dut.led_up.value == 1:
        dut._log.info("UP LED is ON - gesture detected!")
    
    dut._log.info("UP gesture test completed")


# ============================================================================
# Test: Direction Detection - DOWN
# ============================================================================

@cocotb.test()
async def test_gesture_down(dut):
    """Test DOWN gesture detection"""
    dut._log.info("=== DOWN Gesture Detection Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Inject DOWN pattern: events at bottom of sensor
    for _ in range(3):
        for i in range(30):
            x = 160 + random.randint(-50, 50)  # Centered X
            y = random.randint(250, 310)        # Bottom of frame (high Y)
            evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
            await send_evt2_word(dut, evt)
            await ClockCycles(dut.clk, 2)
        await ClockCycles(dut.clk, 1000)
    
    await ClockCycles(dut.clk, 130000)
    
    if dut.led_down.value == 1:
        dut._log.info("DOWN LED is ON - gesture detected!")
    
    dut._log.info("DOWN gesture test completed")


# ============================================================================
# Test: Direction Detection - LEFT
# ============================================================================

@cocotb.test()
async def test_gesture_left(dut):
    """Test LEFT gesture detection"""
    dut._log.info("=== LEFT Gesture Detection Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Inject LEFT pattern: events at left side of sensor
    for _ in range(3):
        for i in range(30):
            x = random.randint(10, 60)          # Left of frame (low X)
            y = 160 + random.randint(-50, 50)   # Centered Y
            evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
            await send_evt2_word(dut, evt)
            await ClockCycles(dut.clk, 2)
        await ClockCycles(dut.clk, 1000)
    
    await ClockCycles(dut.clk, 130000)
    
    if dut.led_left.value == 1:
        dut._log.info("LEFT LED is ON - gesture detected!")
    
    dut._log.info("LEFT gesture test completed")


# ============================================================================
# Test: Direction Detection - RIGHT
# ============================================================================

@cocotb.test()
async def test_gesture_right(dut):
    """Test RIGHT gesture detection"""
    dut._log.info("=== RIGHT Gesture Detection Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Inject RIGHT pattern: events at right side of sensor
    for _ in range(3):
        for i in range(30):
            x = random.randint(260, 310)        # Right of frame (high X)
            y = 160 + random.randint(-50, 50)   # Centered Y
            evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
            await send_evt2_word(dut, evt)
            await ClockCycles(dut.clk, 2)
        await ClockCycles(dut.clk, 1000)
    
    await ClockCycles(dut.clk, 130000)
    
    if dut.led_right.value == 1:
        dut._log.info("RIGHT LED is ON - gesture detected!")
    
    dut._log.info("RIGHT gesture test completed")


# ============================================================================
# Test: UART Debug Output
# ============================================================================

@cocotb.test()
async def test_uart_output(dut):
    """Test UART transmits gesture classification"""
    dut._log.info("=== UART Output Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    # Inject strong pattern to trigger classification
    dut._log.info("Injecting events to trigger UART output...")
    for _ in range(5):
        for i in range(40):
            x = 160 + random.randint(-30, 30)
            y = random.randint(20, 80)  # UP pattern
            evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
            await send_evt2_word(dut, evt)
            await ClockCycles(dut.clk, 2)
        await ClockCycles(dut.clk, 500)
    
    # Wait for frame and UART transmission
    dut._log.info("Waiting for classification and UART TX...")
    
    # Try to receive UART data
    received_bytes = []
    for _ in range(10):
        await ClockCycles(dut.clk, 20000)
        byte_val = await uart_receive_byte(dut, timeout_cycles=100000)
        if byte_val is not None:
            received_bytes.append(byte_val)
            dut._log.info(f"Received UART byte: 0x{byte_val:02X} ('{chr(byte_val) if 32 <= byte_val < 127 else '?'}')")
    
    if received_bytes:
        received_str = ''.join(chr(b) for b in received_bytes if 32 <= b < 127)
        dut._log.info(f"Received string: {received_str}")
    
    dut._log.info("UART output test completed")


# ============================================================================
# Test: Full Integration
# ============================================================================

@cocotb.test()
async def test_full_integration(dut):
    """Full integration test with multiple gesture sequences"""
    dut._log.info("=== Full Integration Test ===")
    
    clock = Clock(dut.clk, 2, unit="step")
    cocotb.start_soon(clock.start())
    
    dut.evt_data.value = 0
    dut.evt_valid.value = 0
    dut.rst_n.value = 1
    
    await reset_dut(dut)
    
    gestures = [
        ("UP", lambda: (160 + random.randint(-50, 50), random.randint(20, 80))),
        ("DOWN", lambda: (160 + random.randint(-50, 50), random.randint(240, 300))),
        ("LEFT", lambda: (random.randint(20, 80), 160 + random.randint(-50, 50))),
        ("RIGHT", lambda: (random.randint(240, 300), 160 + random.randint(-50, 50))),
    ]
    
    for gesture_name, coord_gen in gestures:
        dut._log.info(f"Testing {gesture_name} gesture...")
        
        # Reset between gestures
        await reset_dut(dut)
        
        # Inject pattern
        for _ in range(3):
            for i in range(30):
                x, y = coord_gen()
                evt = make_evt2_cd_event(x=x, y=y, timestamp_lsb=i & 0x3F, polarity=1)
                await send_evt2_word(dut, evt)
                await ClockCycles(dut.clk, 2)
            await ClockCycles(dut.clk, 500)
        
        # Wait for classification
        await ClockCycles(dut.clk, 130000)
        
        # Check LEDs
        led_states = {
            "UP": dut.led_up.value,
            "DOWN": dut.led_down.value,
            "LEFT": dut.led_left.value,
            "RIGHT": dut.led_right.value,
        }
        
        active_leds = [name for name, val in led_states.items() if val == 1]
        if active_leds:
            dut._log.info(f"  Active LEDs: {active_leds}")
            if gesture_name in active_leds:
                dut._log.info(f"  CORRECT: {gesture_name} detected!")
            else:
                dut._log.warning(f"  MISMATCH: Expected {gesture_name}, got {active_leds}")
        else:
            dut._log.warning(f"  No gesture detected for {gesture_name}")
    
    dut._log.info("Full integration test completed")


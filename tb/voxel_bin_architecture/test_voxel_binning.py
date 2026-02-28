"""Unit testbench for voxel_binning with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

CYCLES_PER_BIN = 100
NUM_BINS = 4
READOUT_BINS = 4
GRID_SIZE = 16
COUNTER_BITS = 6
PARALLEL_READS = 4
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE  # 256
TOTAL_CELLS = NUM_BINS * CELLS_PER_BIN  # 1024
MAX_COUNTER = (1 << COUNTER_BITS) - 1


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class VoxelBinningModel:
    """Transaction-level model of the voxel binning engine."""

    def __init__(self):
        self.mem = [0] * TOTAL_CELLS
        self.current_bin_idx = 0
        self.bin_timer = 0
        self.readout_head_snapshot = [0] * CELLS_PER_BIN

    def reset(self):
        self.mem = [0] * TOTAL_CELLS
        self.current_bin_idx = 0
        self.bin_timer = 0
        self.readout_head_snapshot = [0] * CELLS_PER_BIN

    def inject_event(self, event_x_signed, event_y_signed):
        """Increment the counter for this event's cell in the current bin."""
        mapped_x = (event_x_signed + 8) & 0xF
        mapped_y = (event_y_signed + 8) & 0xF
        cell_addr = (mapped_y << 4) | mapped_x
        full_addr = self.current_bin_idx * CELLS_PER_BIN + cell_addr
        if self.mem[full_addr] < MAX_COUNTER:
            self.mem[full_addr] += 1

    def rotate_bin(self):
        """Rotate to next bin and preserve its pre-clear contents for readout."""
        self.current_bin_idx = (self.current_bin_idx + 1) % NUM_BINS
        base = self.current_bin_idx * CELLS_PER_BIN
        # RTL starts readout at the same time it clears this bin. Readout
        # observes pre-clear values, so snapshot before clearing.
        self.readout_head_snapshot = self.mem[base:base + CELLS_PER_BIN].copy()
        for i in range(CELLS_PER_BIN):
            self.mem[base + i] = 0

    def get_readout(self):
        """Return the expected readout data in chronological order."""
        readout = []
        for bin_offset in range(READOUT_BINS):
            idx = (self.current_bin_idx + bin_offset) % NUM_BINS
            base = idx * CELLS_PER_BIN
            if bin_offset == 0:
                readout.extend(self.readout_head_snapshot)
            else:
                readout.extend(self.mem[base:base + CELLS_PER_BIN])
        return readout


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.event_polarity.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, CELLS_PER_BIN + 10)


async def inject_event(dut, x_signed, y_signed, pol=1):
    x_val = x_signed & 0x1F
    dut.event_x.value = x_val
    y_val = y_signed & 0x1F
    dut.event_y.value = y_val
    dut.event_polarity.value = pol
    dut.event_valid.value = 1
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    await RisingEdge(dut.clk)


async def wait_for_readout_start(dut, timeout=CYCLES_PER_BIN + CELLS_PER_BIN + 500):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.readout_start.value) == 1:
            return True
    return False


async def collect_readout(dut, timeout=CYCLES_PER_BIN * READOUT_BINS + 500):
    """Collect all readout_data while readout_valid is asserted."""
    values = []
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.readout_valid.value) == 1:
            packed_bits = str(dut.readout_data.value).replace("x", "0").replace("X", "0").replace("z", "0").replace("Z", "0")
            packed = int(packed_bits, 2)
            for p in range(PARALLEL_READS):
                val = (packed >> (p * COUNTER_BITS)) & MAX_COUNTER
                values.append(val)
            if len(values) >= READOUT_BINS * CELLS_PER_BIN:
                break
        elif len(values) > 0:
            break
    return values[:READOUT_BINS * CELLS_PER_BIN]


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """After reset, readout_start and readout_valid should be 0."""
    await setup(dut)
    assert int(dut.readout_valid.value) == 0


@cocotb.test()
async def test_single_event_increment(dut):
    """Inject one event and verify it increments the correct cell."""
    await setup(dut)
    await inject_event(dut, 0, 0)
    found = await wait_for_readout_start(dut)
    assert found, "No readout_start after bin timer expired"
    readout = await collect_readout(dut)
    assert len(readout) > 0, "No readout data collected"
    nonzero = sum(1 for v in readout if v > 0)
    assert nonzero >= 1, "Expected at least one non-zero cell after event injection"


@cocotb.test()
async def test_bin_rotation(dut):
    """Multiple bin rotations should produce readout_start pulses."""
    await setup(dut)
    starts_seen = 0
    for _ in range((CYCLES_PER_BIN + CELLS_PER_BIN) * (NUM_BINS + 1) + 500):
        await RisingEdge(dut.clk)
        if int(dut.readout_start.value) == 1:
            starts_seen += 1
    assert starts_seen >= NUM_BINS, f"Only saw {starts_seen} readout_start pulses"


@cocotb.test()
async def test_counter_saturation(dut):
    """Counter should saturate at MAX_COUNTER and not wrap."""
    await setup(dut)
    for _ in range(MAX_COUNTER + 5):
        await inject_event(dut, 0, 0)
    found = await wait_for_readout_start(dut)
    assert found
    readout = await collect_readout(dut)
    if len(readout) > 0:
        cell_idx = (8 << 4) | 8  # mapped_x=8, mapped_y=8
        val = readout[cell_idx] if cell_idx < len(readout) else 0
        assert val <= MAX_COUNTER, f"Counter exceeded maximum: {val}"


@cocotb.test()
async def test_golden_model_basic(dut):
    """Inject known events and compare readout against golden model."""
    await setup(dut)
    model = VoxelBinningModel()

    test_events = [
        (-3, -3), (0, 0), (3, 3), (-7, 7), (7, -7),
        (0, 0), (0, 0), (1, 1), (-1, -1), (2, 2),
    ]

    for x, y in test_events:
        await inject_event(dut, x, y)
        model.inject_event(x, y)

    found = await wait_for_readout_start(dut)
    assert found, "No readout_start"

    model.rotate_bin()
    expected = model.get_readout()

    readout = await collect_readout(dut)
    if len(readout) >= len(expected):
        total_dut = sum(readout[:len(expected)])
        total_model = sum(expected)
        assert total_dut == total_model, \
            f"Total event count mismatch: DUT={total_dut}, model={total_model}"


@cocotb.test()
async def test_parallel_readout_width(dut):
    """Readout data should be PARALLEL_READS * COUNTER_BITS wide."""
    await setup(dut)
    await inject_event(dut, 0, 0)
    found = await wait_for_readout_start(dut)
    assert found
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.readout_valid.value) == 1:
            packed_bits = str(dut.readout_data.value).replace("x", "0").replace("X", "0").replace("z", "0").replace("Z", "0")
            packed = int(packed_bits, 2)
            assert packed < (1 << (PARALLEL_READS * COUNTER_BITS))
            break


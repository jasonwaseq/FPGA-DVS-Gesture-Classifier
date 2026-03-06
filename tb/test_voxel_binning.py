"""Robust cocotb testbench for voxel_binning with golden-model scoreboarding."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

CLK_FREQ_HZ = 12_000_000
WINDOW_MS = 400
NUM_BINS = 8
READOUT_BINS = 8
GRID_SIZE = 16
COUNTER_BITS = 16
CELLS_PER_BIN = GRID_SIZE * GRID_SIZE
TOTAL_CELLS = NUM_BINS * CELLS_PER_BIN
FEATURE_COUNT = READOUT_BINS * CELLS_PER_BIN
MAX_COUNTER = (1 << COUNTER_BITS) - 1

BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS  # default-derived: 600000

ST_ACCUM = 0
ST_WAIT_RD = 1
ST_READOUT = 2
ST_CLEAR = 3


class VoxelBinningModel:
    """Transaction-level model matching rtl/voxel_binning.sv defaults."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * TOTAL_CELLS
        self.wr_bin_idx = 0
        self.completed_bins = 0

    def _cell_addr(self, x, y):
        return (y * GRID_SIZE) + x

    def inject_event(self, x, y):
        addr = (self.wr_bin_idx * CELLS_PER_BIN) + self._cell_addr(x, y)
        if self.mem[addr] < MAX_COUNTER:
            self.mem[addr] += 1

    def _readout_snapshot(self):
        start = (self.wr_bin_idx + NUM_BINS - (READOUT_BINS - 1)) % NUM_BINS
        out = []
        for off in range(READOUT_BINS):
            b = (start + off) % NUM_BINS
            base = b * CELLS_PER_BIN
            out.extend(self.mem[base:base + CELLS_PER_BIN])
        return out

    def rotate_bin(self):
        """Apply one timer expiry; return expected readout list or None."""
        next_wr = (self.wr_bin_idx + 1) % NUM_BINS
        completed_next = self.completed_bins + 1
        if completed_next > NUM_BINS:
            completed_next = NUM_BINS

        expected = None
        if completed_next >= READOUT_BINS:
            expected = self._readout_snapshot()

        # Clear next write bin after readout phase.
        base = next_wr * CELLS_PER_BIN
        for i in range(CELLS_PER_BIN):
            self.mem[base + i] = 0

        self.wr_bin_idx = next_wr
        self.completed_bins = completed_next
        return expected


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.event_valid.value = 0
    dut.event_x.value = 0
    dut.event_y.value = 0
    dut.event_polarity.value = 0
    dut.readout_ready.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0

    # Wait for post-reset clear state to return to accumulate.
    await wait_for_state(dut, ST_ACCUM, timeout=5000)


async def wait_for_state(dut, target_state, timeout=10000):
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.state.value) == target_state:
            return
    raise AssertionError(f"Timeout waiting for state {target_state}")


async def inject_event(dut, model, x, y, pol=1):
    assert int(dut.event_ready.value) == 1, "Attempted event inject while event_ready=0"
    dut.event_x.value = x & 0xF
    dut.event_y.value = y & 0xF
    dut.event_polarity.value = pol & 1
    dut.event_valid.value = 1
    model.inject_event(x & 0xF, y & 0xF)
    await RisingEdge(dut.clk)
    dut.event_valid.value = 0
    await RisingEdge(dut.clk)


async def force_timer_rollover(dut):
    assert int(dut.state.value) == ST_ACCUM, "Rollover forcing requires ST_ACCUM"
    dut.timer_ctr.value = CYCLES_PER_BIN_SAFE - 1
    await RisingEdge(dut.clk)


async def collect_readout(dut):
    # Wait for readout_start pulse.
    for _ in range(20000):
        await RisingEdge(dut.clk)
        if int(dut.readout_start.value) == 1:
            break
    else:
        raise AssertionError("Timed out waiting for readout_start")

    values = []
    expected_idx = 0

    for _ in range(FEATURE_COUNT + 2000):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.readout_valid.value):
            idx = int(dut.readout_index.value)
            val = int(dut.readout_data.value)
            assert idx == expected_idx, f"readout_index mismatch: DUT={idx}, expected={expected_idx}"
            values.append(val)
            if int(dut.readout_last.value):
                assert expected_idx == FEATURE_COUNT - 1, "readout_last asserted at wrong index"
                break
            expected_idx += 1
        await NextTimeStep()

    assert len(values) == FEATURE_COUNT, f"Readout length {len(values)} != {FEATURE_COUNT}"
    return values


async def rotate_and_check(dut, model, tag):
    expected = model.rotate_bin()
    await force_timer_rollover(dut)

    if expected is None:
        await wait_for_state(dut, ST_ACCUM)
        return

    got = await collect_readout(dut)
    assert got == expected, f"{tag}: readout mismatch"
    await wait_for_state(dut, ST_ACCUM)


@cocotb.test()
async def test_reset_and_event_ready(dut):
    await setup(dut)
    assert int(dut.event_ready.value) == 1
    assert int(dut.readout_valid.value) == 0


@cocotb.test()
async def test_known_events_then_readout(dut):
    await setup(dut)
    model = VoxelBinningModel()

    events = [(0, 0), (8, 8), (15, 15), (8, 8), (3, 9), (3, 9), (3, 9)]
    for x, y in events:
        await inject_event(dut, model, x, y)

    for i in range(READOUT_BINS):
        await rotate_and_check(dut, model, f"known-{i}")


@cocotb.test()
async def test_wait_rd_backpressure(dut):
    await setup(dut)
    model = VoxelBinningModel()

    # Advance to first readout-eligible boundary.
    for _ in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, "prefill")

    # Hold readout_ready low at rollover; should park in ST_WAIT_RD.
    dut.readout_ready.value = 0
    expected = model.rotate_bin()
    await force_timer_rollover(dut)
    assert expected is not None
    assert int(dut.state.value) in (ST_WAIT_RD, ST_READOUT)

    saw_start_while_low = False
    for _ in range(50):
        await RisingEdge(dut.clk)
        if int(dut.readout_start.value):
            saw_start_while_low = True
            break
    assert not saw_start_while_low, "readout_start asserted despite readout_ready=0"

    dut.readout_ready.value = 1
    got = await collect_readout(dut)
    assert got == expected, "Backpressured readout mismatch"
    await wait_for_state(dut, ST_ACCUM)


@cocotb.test()
async def test_counter_saturation(dut):
    await setup(dut)
    model = VoxelBinningModel()

    hot_x, hot_y = 5, 6
    hot_addr = (model.wr_bin_idx * CELLS_PER_BIN) + (hot_y * GRID_SIZE + hot_x)

    # Seed near max directly, then add events to verify clamp at MAX_COUNTER.
    dut.mem[hot_addr].value = MAX_COUNTER - 1
    model.mem[hot_addr] = MAX_COUNTER - 1

    await inject_event(dut, model, hot_x, hot_y)
    await inject_event(dut, model, hot_x, hot_y)

    for i in range(READOUT_BINS):
        await rotate_and_check(dut, model, f"sat-{i}")


@cocotb.test()
async def test_events_ignored_when_not_accum(dut):
    await setup(dut)
    model = VoxelBinningModel()

    # Trigger rollover to enter clear/readout path (not accum).
    for _ in range(READOUT_BINS - 1):
        await rotate_and_check(dut, model, "pre")

    expected = model.rotate_bin()
    await force_timer_rollover(dut)
    assert expected is not None

    # While not in ST_ACCUM, pulse event_valid; model does not accept this event.
    if int(dut.state.value) != ST_ACCUM:
        dut.event_x.value = 9
        dut.event_y.value = 9
        dut.event_polarity.value = 1
        dut.event_valid.value = 1
        await RisingEdge(dut.clk)
        dut.event_valid.value = 0

    got = await collect_readout(dut)
    assert got == expected, "Non-accum event unexpectedly affected readout"
    await wait_for_state(dut, ST_ACCUM)


@cocotb.test()
async def test_randomized_multibin_scoreboard(dut):
    await setup(dut)
    model = VoxelBinningModel()
    rng = random.Random(0xB1A5)

    for bin_idx in range(10):
        events = rng.randint(0, 40)
        for _ in range(events):
            x = rng.randint(0, GRID_SIZE - 1)
            y = rng.randint(0, GRID_SIZE - 1)
            pol = rng.randint(0, 1)
            await inject_event(dut, model, x, y, pol)

        await rotate_and_check(dut, model, f"rnd-{bin_idx}")

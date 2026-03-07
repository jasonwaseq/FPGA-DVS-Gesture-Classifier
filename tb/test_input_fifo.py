"""Robust cocotb testbench for input_fifo with cycle-accurate golden model."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

WIDTH = 32
DEPTH_LOG2 = 8
DEPTH = 1 << DEPTH_LOG2
PTR_MASK = (1 << (DEPTH_LOG2 + 1)) - 1
ADDR_MASK = (1 << DEPTH_LOG2) - 1


class InputFifoModel:
    """Cycle-accurate model for rtl/input_fifo.sv default parameters."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.wr_ptr = 0
        self.rd_ptr = 0
        self.ram = [0] * DEPTH

    @property
    def full(self):
        return int(((self.wr_ptr >> DEPTH_LOG2) ^ (self.rd_ptr >> DEPTH_LOG2)) and
                   ((self.wr_ptr & ADDR_MASK) == (self.rd_ptr & ADDR_MASK)))

    @property
    def empty(self):
        return int((not ((self.wr_ptr >> DEPTH_LOG2) ^ (self.rd_ptr >> DEPTH_LOG2))) and
                   ((self.wr_ptr & ADDR_MASK) == (self.rd_ptr & ADDR_MASK)))

    def outputs(self):
        ready_o = 0 if self.full else 1
        valid_o = 0 if self.empty else 1
        data_o = self.ram[self.rd_ptr & ADDR_MASK]
        return ready_o, valid_o, data_o & ((1 << WIDTH) - 1)

    def step(self, reset_i, valid_i, data_i, ready_i):
        # Combinational values from pre-edge state.
        ready_o_pre, valid_o_pre, _ = self.outputs()
        wr_en = int(valid_i and ready_o_pre)
        rd_en = int(valid_o_pre and ready_i)

        if reset_i:
            self.wr_ptr = 0
            self.rd_ptr = 0
        else:
            if wr_en:
                self.ram[self.wr_ptr & ADDR_MASK] = data_i & ((1 << WIDTH) - 1)
                self.wr_ptr = (self.wr_ptr + 1) & PTR_MASK

            if rd_en:
                self.rd_ptr = (self.rd_ptr + 1) & PTR_MASK

        return self.outputs()


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, unit="ns").start())
    dut.reset_i.value = 1
    dut.valid_i.value = 0
    dut.data_i.value = 0
    dut.ready_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def drive_and_check(dut, model, reset_i, valid_i, data_i, ready_i, tag):
    dut.reset_i.value = reset_i
    dut.valid_i.value = valid_i
    dut.data_i.value = data_i
    dut.ready_i.value = ready_i

    exp_ready, exp_valid, exp_data = model.step(reset_i, valid_i, data_i, ready_i)

    await RisingEdge(dut.clk_i)
    await ReadOnly()

    got_ready = int(dut.ready_o.value)
    got_valid = int(dut.valid_o.value)
    got_data = int(dut.data_o.value)

    assert got_ready == exp_ready, f"{tag}: ready_o DUT={got_ready} model={exp_ready}"
    assert got_valid == exp_valid, f"{tag}: valid_o DUT={got_valid} model={exp_valid}"
    if exp_valid:
        assert got_data == exp_data, f"{tag}: data_o DUT=0x{got_data:08X} model=0x{exp_data:08X}"

    await NextTimeStep()


@cocotb.test()
async def test_reset_and_empty_flags(dut):
    await setup(dut)
    assert int(dut.valid_o.value) == 0
    assert int(dut.ready_o.value) == 1


@cocotb.test()
async def test_basic_ordering(dut):
    await setup(dut)
    model = InputFifoModel()

    # Bring model through same reset/deassert sequence used by setup.
    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    payload = [0x11, 0x22, 0x33, 0x44]

    for i, p in enumerate(payload):
        await drive_and_check(dut, model, 0, 1, p, 0, f"wr-{i}")

    observed = []
    for _ in range(128):
        if int(dut.valid_o.value):
            observed.append(int(dut.data_o.value))
        await drive_and_check(dut, model, 0, 0, 0, 1, f"rd-{len(observed)}")
        if len(observed) >= len(payload):
            break

    assert len(observed) >= len(payload), f"Timed out draining payload, observed={observed}"
    assert observed[:4] == payload, f"FIFO ordering mismatch: {observed[:4]} vs {payload}"


@cocotb.test()
async def test_full_and_overflow_drop(dut):
    await setup(dut)
    model = InputFifoModel()

    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    # Fill FIFO to capacity.
    for i in range(DEPTH):
        await drive_and_check(dut, model, 0, 1, i, 0, f"fill-{i}")

    assert int(dut.ready_o.value) == 0, "ready_o should deassert when full"

    # Attempt overflow writes while full; queue content should remain unchanged.
    for i in range(8):
        await drive_and_check(dut, model, 0, 1, 0xBAD00000 + i, 0, f"ovf-{i}")

    # Drain and check first values are original fill data.
    drained = []
    for _ in range(DEPTH * 4):
        if int(dut.valid_o.value):
            drained.append(int(dut.data_o.value))
        await drive_and_check(dut, model, 0, 0, 0, 1, f"drain-{len(drained)}")
        if len(drained) >= DEPTH:
            break

    assert len(drained) >= DEPTH, f"Timed out draining full FIFO, drained={len(drained)}"
    assert drained[:8] == list(range(8)), "Overflow attempts corrupted FIFO head"


@cocotb.test()
async def test_randomized_cycle_scoreboard(dut):
    await setup(dut)
    model = InputFifoModel()
    rng = random.Random(0xF1F0F1F0)

    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    for cycle in range(2500):
        valid_i = rng.randint(0, 1)
        ready_i = rng.randint(0, 1)
        data_i = rng.getrandbits(WIDTH)
        await drive_and_check(dut, model, 0, valid_i, data_i, ready_i, f"rnd-{cycle}")


@cocotb.test()
async def test_simultaneous_read_write_stress(dut):
    await setup(dut)
    model = InputFifoModel()

    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    # Prime with data then run many cycles with both ends active.
    for i in range(32):
        await drive_and_check(dut, model, 0, 1, 0x1000 + i, 0, f"prime-{i}")

    for cycle in range(500):
        await drive_and_check(dut, model, 0, 1, 0x2000 + cycle, 1, f"rw-{cycle}")


@cocotb.test()
async def test_single_element_round_trip(dut):
    """Write one item, then read it back; FIFO must go empty→non-empty→empty."""
    await setup(dut)
    model = InputFifoModel()

    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    # Initially empty.
    assert int(dut.valid_o.value) == 0, "Expected empty FIFO after reset"

    # Write one word.
    payload = 0xDEADBEEF
    await drive_and_check(dut, model, 0, 1, payload, 0, "write-one")

    # FIFO should now be non-empty.
    assert int(dut.valid_o.value) == 1, "FIFO should be non-empty after one write"

    # Read it back.
    got = int(dut.data_o.value)
    assert got == payload, f"Round-trip mismatch: got 0x{got:08X}, expected 0x{payload:08X}"
    await drive_and_check(dut, model, 0, 0, 0, 1, "read-one")

    # FIFO must be empty again.
    assert int(dut.valid_o.value) == 0, "FIFO should be empty after reading the only item"


@cocotb.test()
async def test_fill_drain_cycle(dut):
    """Fill FIFO to capacity, drain it completely, fill again; verify both drains match."""
    await setup(dut)
    model = InputFifoModel()

    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    def fill_once(base):
        return [base + i for i in range(DEPTH)]

    async def fill(base):
        for v in fill_once(base):
            await drive_and_check(dut, model, 0, 1, v, 0, f"fill-0x{v:08X}")
        assert int(dut.ready_o.value) == 0, "FIFO should be full"

    async def drain():
        drained = []
        for _ in range(DEPTH * 2):
            if int(dut.valid_o.value):
                drained.append(int(dut.data_o.value))
            await drive_and_check(dut, model, 0, 0, 0, 1, f"drain-{len(drained)}")
            if len(drained) >= DEPTH:
                break
        return drained

    # First fill-drain cycle.
    await fill(0xA000)
    first_drain = await drain()
    assert first_drain == fill_once(0xA000), "First drain content mismatch"
    assert int(dut.valid_o.value) == 0, "FIFO not empty after full drain"

    # Second fill-drain cycle.
    await fill(0xB000)
    second_drain = await drain()
    assert second_drain == fill_once(0xB000), "Second drain content mismatch"
    assert int(dut.valid_o.value) == 0, "FIFO not empty after second full drain"


@cocotb.test()
async def test_mid_reset_clears_pending_data(dut):
    """Write several items, assert reset mid-stream, verify FIFO is empty after reset."""
    await setup(dut)
    model = InputFifoModel()

    for _ in range(5):
        model.step(1, 0, 0, 0)
    for _ in range(2):
        model.step(0, 0, 0, 0)

    # Write 16 items.
    for i in range(16):
        await drive_and_check(dut, model, 0, 1, 0xC000 + i, 0, f"pre-{i}")

    # Assert reset.
    for _ in range(4):
        await drive_and_check(dut, model, 1, 0, 0, 0, "rst-mid")

    # After reset: FIFO must be empty and ready.
    assert int(dut.valid_o.value) == 0, "FIFO not empty after mid-stream reset"
    assert int(dut.ready_o.value) == 1, "FIFO not ready after reset"

    # Verify the FIFO works normally after reset.
    await drive_and_check(dut, model, 0, 1, 0xFACE_CAFE, 0, "post-rst-wr")
    assert int(dut.data_o.value) == 0xFACE_CAFE, \
        f"Post-reset data mismatch: 0x{int(dut.data_o.value):08X}"
    await drive_and_check(dut, model, 0, 0, 0, 1, "post-rst-rd")

"""Robust cocotb testbench for input_fifo with cycle-accurate golden model."""

from collections import deque
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
        self.rd_data_l = 0
        self.data_l = 0
        self.trail = 0
        self.firstwrite = 0
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
        data_o = self.data_l if (self.firstwrite or self.trail) else self.rd_data_l
        return ready_o, valid_o, data_o & ((1 << WIDTH) - 1)

    def step(self, reset_i, valid_i, data_i, ready_i):
        # Combinational values from pre-edge state.
        ready_o_pre, valid_o_pre, _ = self.outputs()
        wr_en = int(valid_i and ready_o_pre)
        rd_en = int(valid_o_pre and ready_i)
        mux = (self.rd_ptr + 1) if rd_en else self.rd_ptr
        mux &= PTR_MASK

        # Snapshot pre-edge state for NBA behavior.
        old_wr_ptr = self.wr_ptr
        old_rd_ptr = self.rd_ptr
        old_firstwrite = self.firstwrite
        old_ram = self.ram.copy()

        if reset_i:
            self.wr_ptr = 0
            self.rd_ptr = 0
            self.data_l = 0
            self.trail = 0
            self.firstwrite = 0
            self.rd_data_l = 0
        else:
            # rd_data_l samples memory every cycle (rd_valid_i hardwired to 1).
            self.rd_data_l = old_ram[mux & ADDR_MASK]

            if wr_en:
                self.wr_ptr = (old_wr_ptr + 1) & PTR_MASK
                self.data_l = data_i & ((1 << WIDTH) - 1)
            else:
                self.wr_ptr = old_wr_ptr

            if rd_en:
                self.rd_ptr = (old_rd_ptr + 1) & PTR_MASK
            else:
                self.rd_ptr = old_rd_ptr

            # Write after read sample in same edge (matches NBAs).
            if wr_en:
                self.ram[old_wr_ptr & ADDR_MASK] = data_i & ((1 << WIDTH) - 1)

            pre_empty = int((old_wr_ptr == old_rd_ptr))
            self.firstwrite = int((wr_en and pre_empty) or (old_firstwrite and rd_en))
            self.trail = int(((mux == old_wr_ptr) and rd_en))

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
    while len(observed) < len(payload):
        await drive_and_check(dut, model, 0, 0, 0, 1, f"rd-{len(observed)}")
        if int(dut.valid_o.value):
            observed.append(int(dut.data_o.value))

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
    while len(drained) < DEPTH:
        await drive_and_check(dut, model, 0, 0, 0, 1, f"drain-{len(drained)}")
        if int(dut.valid_o.value):
            drained.append(int(dut.data_o.value))

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

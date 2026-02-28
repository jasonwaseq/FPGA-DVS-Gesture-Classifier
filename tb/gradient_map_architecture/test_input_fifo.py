"""Unit testbench for input_fifo with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

DEPTH = 8
PTR_BITS = 3
DATA_WIDTH = 32


# ---------------------------------------------------------------------------
# Golden reference model (identical logic to voxel_bin FIFO)
# ---------------------------------------------------------------------------
class FifoModel:
    def __init__(self, depth=DEPTH, ptr_bits=PTR_BITS):
        self.depth = depth
        self.ptr_bits = ptr_bits
        self.reset()

    def reset(self):
        self.mem = [0] * self.depth
        self.wr_ptr = 0
        self.rd_ptr = 0
        self.rd_data = 0

    @property
    def full(self):
        msb = 1 << self.ptr_bits
        low = msb - 1
        return ((self.wr_ptr ^ self.rd_ptr) == msb) and \
               ((self.wr_ptr & low) == (self.rd_ptr & low))

    @property
    def empty(self):
        return self.wr_ptr == self.rd_ptr

    @property
    def count(self):
        return (self.wr_ptr - self.rd_ptr) & ((1 << (self.ptr_bits + 1)) - 1)

    def step(self, wr_en, wr_data, rd_en):
        do_write = wr_en and not self.full
        do_read = rd_en and not self.empty
        if do_write:
            self.mem[self.wr_ptr & ((1 << self.ptr_bits) - 1)] = wr_data
            self.wr_ptr = (self.wr_ptr + 1) & ((1 << (self.ptr_bits + 1)) - 1)
        self.rd_data = self.mem[self.rd_ptr & ((1 << self.ptr_bits) - 1)]
        if do_read:
            self.rd_ptr = (self.rd_ptr + 1) & ((1 << (self.ptr_bits + 1)) - 1)
        return self.rd_data, self.empty, self.full, self.count


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.wr_en.value = 0
    dut.wr_data.value = 0
    dut.rd_en.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_reset(dut):
    await setup(dut)
    assert int(dut.empty.value) == 1
    assert int(dut.full.value) == 0
    assert int(dut.count.value) == 0


@cocotb.test()
async def test_write_read_cycle(dut):
    await setup(dut)
    dut.wr_en.value = 1
    dut.wr_data.value = 0xCAFEBABE
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.empty.value) == 0

    dut.rd_en.value = 1
    await RisingEdge(dut.clk)
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.rd_data.value) == 0xCAFEBABE


@cocotb.test()
async def test_fill_drain(dut):
    await setup(dut)
    for i in range(DEPTH):
        dut.wr_en.value = 1
        dut.wr_data.value = (i + 1) * 0x111
        await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.full.value) == 1

    read_vals = []
    for _ in range(DEPTH):
        dut.rd_en.value = 1
        await RisingEdge(dut.clk)
        await RisingEdge(dut.clk)
        read_vals.append(int(dut.rd_data.value))
    dut.rd_en.value = 0
    assert read_vals == [(i + 1) * 0x111 for i in range(DEPTH)]


@cocotb.test()
async def test_overflow_protection(dut):
    await setup(dut)
    for i in range(DEPTH + 2):
        dut.wr_en.value = 1
        dut.wr_data.value = i
        await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)
    assert int(dut.count.value) == DEPTH


@cocotb.test()
async def test_golden_random(dut):
    await setup(dut)
    model = FifoModel()
    for _ in range(300):
        wr = random.randint(0, 1)
        rd = random.randint(0, 1)
        wd = random.randint(0, (1 << DATA_WIDTH) - 1)
        dut.wr_en.value = wr
        dut.wr_data.value = wd
        dut.rd_en.value = rd
        await RisingEdge(dut.clk)
        model.step(wr, wd, rd)

    await RisingEdge(dut.clk)
    assert int(dut.empty.value) == (1 if model.empty else 0)
    assert int(dut.full.value) == (1 if model.full else 0)
    assert int(dut.count.value) == model.count

"""Unit testbench for input_fifo with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

DEPTH = 8
PTR_BITS = 3
DATA_WIDTH = 32


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class FifoModel:
    """Cycle-accurate model of the synchronous ring-buffer FIFO."""

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
        msb_mask = 1 << self.ptr_bits
        low_mask = msb_mask - 1
        return ((self.wr_ptr ^ self.rd_ptr) == msb_mask) and \
               ((self.wr_ptr & low_mask) == (self.rd_ptr & low_mask))

    @property
    def empty(self):
        return self.wr_ptr == self.rd_ptr

    @property
    def count(self):
        return (self.wr_ptr - self.rd_ptr) & ((1 << (self.ptr_bits + 1)) - 1)

    def step(self, wr_en, wr_data, rd_en):
        """One clock cycle. Returns (rd_data, empty, full, count) after the edge."""
        do_write = wr_en and not self.full
        do_read = rd_en and not self.empty

        if do_write:
            wr_addr = self.wr_ptr & ((1 << self.ptr_bits) - 1)
            self.mem[wr_addr] = wr_data
            self.wr_ptr = (self.wr_ptr + 1) & ((1 << (self.ptr_bits + 1)) - 1)

        rd_addr = self.rd_ptr & ((1 << self.ptr_bits) - 1)
        self.rd_data = self.mem[rd_addr]
        if do_read:
            self.rd_ptr = (self.rd_ptr + 1) & ((1 << (self.ptr_bits + 1)) - 1)

        return self.rd_data, self.empty, self.full, self.count


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.wr_en.value = 0
    dut.wr_data.value = 0
    dut.rd_en.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """FIFO should be empty after reset."""
    await setup(dut)
    assert int(dut.empty.value) == 1
    assert int(dut.full.value) == 0
    assert int(dut.count.value) == 0


@cocotb.test()
async def test_single_write_read(dut):
    """Write one word, read it back."""
    await setup(dut)
    dut.wr_en.value = 1
    dut.wr_data.value = 0xDEADBEEF
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.empty.value) == 0
    assert int(dut.count.value) == 1

    dut.rd_en.value = 1
    await RisingEdge(dut.clk)
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.rd_data.value) == 0xDEADBEEF
    assert int(dut.empty.value) == 1


@cocotb.test()
async def test_fill_to_capacity(dut):
    """Fill FIFO to DEPTH, verify full flag, then drain."""
    await setup(dut)

    for i in range(DEPTH):
        dut.wr_en.value = 1
        dut.wr_data.value = i + 1
        await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.full.value) == 1
    assert int(dut.count.value) == DEPTH

    read_values = []
    for i in range(DEPTH):
        dut.rd_en.value = 1
        await RisingEdge(dut.clk)
        dut.rd_en.value = 0
        await RisingEdge(dut.clk)
        read_values.append(int(dut.rd_data.value))

    assert read_values == list(range(1, DEPTH + 1)), f"Read back: {read_values}"


@cocotb.test()
async def test_overflow_protection(dut):
    """Writes to a full FIFO should be silently dropped."""
    await setup(dut)

    for i in range(DEPTH):
        dut.wr_en.value = 1
        dut.wr_data.value = 0xAA
        await RisingEdge(dut.clk)

    dut.wr_data.value = 0xBAD
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.full.value) == 1
    assert int(dut.count.value) == DEPTH


@cocotb.test()
async def test_simultaneous_read_write(dut):
    """Read and write in the same cycle when FIFO has data."""
    await setup(dut)

    dut.wr_en.value = 1
    dut.wr_data.value = 0x11
    await RisingEdge(dut.clk)
    dut.wr_data.value = 0x22
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    await RisingEdge(dut.clk)

    dut.wr_en.value = 1
    dut.rd_en.value = 1
    dut.wr_data.value = 0x33
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)

    assert int(dut.count.value) == 2


@cocotb.test()
async def test_golden_model_random(dut):
    """Random write/read sequences compared against golden model."""
    await setup(dut)
    model = FifoModel(DEPTH, PTR_BITS)

    for _ in range(200):
        wr_en = random.randint(0, 1)
        rd_en = random.randint(0, 1)
        wr_data = random.randint(0, (1 << DATA_WIDTH) - 1)

        dut.wr_en.value = wr_en
        dut.wr_data.value = wr_data
        dut.rd_en.value = rd_en
        await RisingEdge(dut.clk)

        m_rd, m_empty, m_full, m_count = model.step(wr_en, wr_data, rd_en)

    await RisingEdge(dut.clk)
    assert int(dut.empty.value) == (1 if model.empty else 0), \
        f"empty: DUT={int(dut.empty.value)}, model={model.empty}"
    assert int(dut.full.value) == (1 if model.full else 0), \
        f"full: DUT={int(dut.full.value)}, model={model.full}"
    assert int(dut.count.value) == model.count, \
        f"count: DUT={int(dut.count.value)}, model={model.count}"


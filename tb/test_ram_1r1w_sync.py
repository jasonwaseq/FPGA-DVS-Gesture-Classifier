"""Robust cocotb testbench for ram_1r1w_sync with cycle-accurate golden model."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

WIDTH = 32
DEPTH = 512
ADDR_MASK = DEPTH - 1
DATA_MASK = (1 << WIDTH) - 1


class Ram1R1WSyncModel:
    """Cycle-accurate model for rtl/ram_1r1w_sync.sv default params."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.mem = [0] * DEPTH
        self.rd_data = 0

    def step(self, reset_i, wr_valid_i, wr_data_i, wr_addr_i, rd_valid_i, rd_addr_i):
        old_mem = self.mem.copy()

        if reset_i:
            self.rd_data = 0
        elif rd_valid_i:
            self.rd_data = old_mem[rd_addr_i & ADDR_MASK]

        if wr_valid_i:
            self.mem[wr_addr_i & ADDR_MASK] = wr_data_i & DATA_MASK

        return self.rd_data


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk_i, 10, unit="ns").start())
    dut.reset_i.value = 1
    dut.wr_valid_i.value = 0
    dut.wr_data_i.value = 0
    dut.wr_addr_i.value = 0
    dut.rd_valid_i.value = 0
    dut.rd_addr_i.value = 0
    await ClockCycles(dut.clk_i, 5)
    dut.reset_i.value = 0
    await ClockCycles(dut.clk_i, 2)


async def drive_and_check(dut, model, reset_i, wr_valid_i, wr_data_i, wr_addr_i, rd_valid_i, rd_addr_i, tag):
    dut.reset_i.value = reset_i
    dut.wr_valid_i.value = wr_valid_i
    dut.wr_data_i.value = wr_data_i
    dut.wr_addr_i.value = wr_addr_i
    dut.rd_valid_i.value = rd_valid_i
    dut.rd_addr_i.value = rd_addr_i

    exp = model.step(reset_i, wr_valid_i, wr_data_i, wr_addr_i, rd_valid_i, rd_addr_i)

    await RisingEdge(dut.clk_i)
    await ReadOnly()

    got = int(dut.rd_data_o.value)
    assert got == exp, f"{tag}: rd_data_o DUT=0x{got:08X} model=0x{exp:08X}"

    await NextTimeStep()


@cocotb.test()
async def test_reset_and_basic_read_write(dut):
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    await drive_and_check(dut, model, 0, 1, 0xDEADBEEF, 7, 0, 0, "write")
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 7, "read")
    assert int(dut.rd_data_o.value) == 0xDEADBEEF


@cocotb.test()
async def test_read_before_write_same_cycle_same_addr(dut):
    await setup(dut)
    model = Ram1R1WSyncModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    # Prime location.
    await drive_and_check(dut, model, 0, 1, 0x11111111, 12, 0, 0, "prime")

    # Same-cycle read and write same addr: rd_data should return old value.
    await drive_and_check(dut, model, 0, 1, 0x22222222, 12, 1, 12, "rw-same")
    assert int(dut.rd_data_o.value) == 0x11111111

    # Next read sees new value.
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 12, "read-new")
    assert int(dut.rd_data_o.value) == 0x22222222


@cocotb.test()
async def test_write_during_reset(dut):
    """RTL keeps writes active during reset; verify this edge behavior."""
    await setup(dut)
    model = Ram1R1WSyncModel()

    # Keep reset high and write.
    await drive_and_check(dut, model, 1, 1, 0xA5A5A5A5, 33, 0, 0, "wr-in-rst")

    # Deassert reset and read back written value.
    await drive_and_check(dut, model, 0, 0, 0, 0, 1, 33, "read-after-rst")
    assert int(dut.rd_data_o.value) == 0xA5A5A5A5


@cocotb.test()
async def test_randomized_golden_scoreboard(dut):
    await setup(dut)
    model = Ram1R1WSyncModel()
    rng = random.Random(0xAA551234)

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 0, 0, 0, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 0, 0, 0, "idle")

    for cycle in range(3000):
        # Mostly out of reset, with occasional reset pulses.
        reset_i = 1 if rng.randint(0, 199) == 0 else 0
        wr_valid_i = rng.randint(0, 1)
        rd_valid_i = rng.randint(0, 1)
        wr_data_i = rng.getrandbits(WIDTH)
        wr_addr_i = rng.randint(0, DEPTH - 1)
        rd_addr_i = rng.randint(0, DEPTH - 1)

        await drive_and_check(
            dut,
            model,
            reset_i,
            wr_valid_i,
            wr_data_i,
            wr_addr_i,
            rd_valid_i,
            rd_addr_i,
            f"rnd-{cycle}",
        )

"""Robust cocotb testbench for systolic_array with signed matmul golden model."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

N = 16
DATA_BITS = 16
ACC_BITS = (2 * DATA_BITS) + 4 + 1  # PRODUCT(32) + clog2(16)=4 + 1 = 37
TOTAL_CYCLES = (3 * N) - 2
DATA_MASK = (1 << DATA_BITS) - 1
ACC_MASK = (1 << ACC_BITS) - 1


def to_signed(v, bits):
    v &= (1 << bits) - 1
    if v & (1 << (bits - 1)):
        return v - (1 << bits)
    return v


def from_signed(v, bits):
    return v & ((1 << bits) - 1)


def pack_matrix(mat, bits):
    packed = 0
    idx = 0
    for i in range(N):
        for j in range(N):
            packed |= from_signed(mat[i][j], bits) << (idx * bits)
            idx += 1
    return packed


def unpack_matrix(packed, bits):
    mat = [[0 for _ in range(N)] for _ in range(N)]
    idx = 0
    for i in range(N):
        for j in range(N):
            raw = (packed >> (idx * bits)) & ((1 << bits) - 1)
            mat[i][j] = to_signed(raw, bits)
            idx += 1
    return mat


def golden_matmul(a, b):
    out = [[0 for _ in range(N)] for _ in range(N)]
    for i in range(N):
        for j in range(N):
            acc = 0
            for k in range(N):
                acc += int(a[i][k]) * int(b[k][j])
            out[i][j] = to_signed(acc, ACC_BITS)
    return out


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.reset.value = 1
    dut.start.value = 0
    dut.A_matrix_flat.value = 0
    dut.B_matrix_flat.value = 0
    await ClockCycles(dut.clk, 5)
    dut.reset.value = 0
    await ClockCycles(dut.clk, 2)


async def run_mul(dut, a, b, tag):
    dut.A_matrix_flat.value = pack_matrix(a, DATA_BITS)
    dut.B_matrix_flat.value = pack_matrix(b, DATA_BITS)

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # busy should assert after start acceptance.
    await ReadOnly()
    assert int(dut.busy.value) == 1, f"{tag}: busy did not assert after start"

    saw_done = False
    for cyc in range(TOTAL_CYCLES + 20):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.done.value):
            saw_done = True
            assert int(dut.busy.value) == 0, f"{tag}: busy should drop when done=1"
            break
        else:
            assert int(dut.busy.value) == 1, f"{tag}: busy dropped early at cycle {cyc}"

    assert saw_done, f"{tag}: timed out waiting for done"

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    exp = golden_matmul(a, b)
    assert got == exp, f"{tag}: output matrix mismatch"

    # done is one-cycle pulse.
    await RisingEdge(dut.clk)
    assert int(dut.done.value) == 0, f"{tag}: done not a single-cycle pulse"

    await NextTimeStep()


@cocotb.test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.busy.value) == 0
    assert int(dut.done.value) == 0


@cocotb.test()
async def test_zero_matrices(dut):
    await setup(dut)
    a = [[0 for _ in range(N)] for _ in range(N)]
    b = [[0 for _ in range(N)] for _ in range(N)]
    await run_mul(dut, a, b, "zeros")


@cocotb.test()
async def test_identity_times_random(dut):
    await setup(dut)
    rng = random.Random(0x1D31F17)

    a = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    b = [[rng.randint(-200, 200) for _ in range(N)] for _ in range(N)]

    await run_mul(dut, a, b, "identity")


@cocotb.test()
async def test_start_ignored_while_running(dut):
    await setup(dut)

    a = [[1 if i == j else 0 for j in range(N)] for i in range(N)]
    b = [[2 if i == j else 0 for j in range(N)] for i in range(N)]

    dut.A_matrix_flat.value = pack_matrix(a, DATA_BITS)
    dut.B_matrix_flat.value = pack_matrix(b, DATA_BITS)

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # Pulse start again while busy; DUT should ignore it.
    await ClockCycles(dut.clk, 5)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    saw_done = False
    for _ in range(TOTAL_CYCLES + 20):
        await RisingEdge(dut.clk)
        if int(dut.done.value):
            saw_done = True
            break

    assert saw_done, "Did not complete first run"

    got = unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BITS)
    exp = golden_matmul(a, b)
    assert got == exp, "Unexpected restart/perturbation from mid-run start pulse"


@cocotb.test()
async def test_randomized_golden(dut):
    await setup(dut)
    rng = random.Random(0x5A57A11C)

    for trial in range(10):
        a = [[rng.randint(-300, 300) for _ in range(N)] for _ in range(N)]
        b = [[rng.randint(-300, 300) for _ in range(N)] for _ in range(N)]
        await run_mul(dut, a, b, f"rnd-{trial}")
        await ClockCycles(dut.clk, 2)

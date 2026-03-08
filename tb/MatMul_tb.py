"""Standalone cocotb matmul tests for rtl/systolic_array.sv."""

import math

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from util.test_logging import logged_test

N = 4
DATA_BIT_SIZE = 16
ACC_BIT_SIZE = (2 * DATA_BIT_SIZE) + (N - 1).bit_length()
TOTAL_CYCLES = (3 * N) - 1
DATA_MASK = (1 << DATA_BIT_SIZE) - 1
ACC_MASK = (1 << ACC_BIT_SIZE) - 1


def configure_from_dut(dut):
    global N, DATA_BIT_SIZE, ACC_BIT_SIZE, TOTAL_CYCLES, DATA_MASK, ACC_MASK

    if hasattr(dut, "DATA_BIT_SIZE"):
        DATA_BIT_SIZE = int(dut.DATA_BIT_SIZE.value)

    n_sq = len(dut.A_matrix_flat) // DATA_BIT_SIZE
    n = math.isqrt(n_sq)
    assert n * n == n_sq, f"Non-square flattening: bits={len(dut.A_matrix_flat)}"
    N = n

    acc_bits = len(dut.Out_matrix_flat) // (N * N)
    ACC_BIT_SIZE = acc_bits
    TOTAL_CYCLES = (3 * N) - 1
    DATA_MASK = (1 << DATA_BIT_SIZE) - 1
    ACC_MASK = (1 << ACC_BIT_SIZE) - 1


def matmul_expected(a, b):
    c = [[0 for _ in range(N)] for _ in range(N)]
    for i in range(N):
        for j in range(N):
            total = 0
            for k in range(N):
                total += int(a[i][k]) * int(b[k][j])
            c[i][j] = total & ACC_MASK
    return c


def pack_matrix(mat, elem_bits):
    packed = 0
    mask = (1 << elem_bits) - 1
    for i in range(N):
        for j in range(N):
            idx = i * N + j
            packed |= (mat[i][j] & mask) << (idx * elem_bits)
    return packed


def unpack_matrix(packed, elem_bits):
    mat = [[0 for _ in range(N)] for _ in range(N)]
    mask = (1 << elem_bits) - 1
    for i in range(N):
        for j in range(N):
            idx = i * N + j
            mat[i][j] = (packed >> (idx * elem_bits)) & mask
    return mat


def build_incremental_matrix(offset):
    val = offset
    out = []
    for _ in range(N):
        row = []
        for _ in range(N):
            row.append(val & DATA_MASK)
            val += 1
        out.append(row)
    return out


async def setup_dut(dut):
    configure_from_dut(dut)
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())

    dut.reset.value = 1
    dut.start.value = 0
    dut.A_matrix_flat.value = 0
    dut.B_matrix_flat.value = 0
    await ClockCycles(dut.clk, 5)
    dut.reset.value = 0
    await ClockCycles(dut.clk, 2)


async def run_matmul(dut, a, b):
    dut.A_matrix_flat.value = pack_matrix(a, DATA_BIT_SIZE)
    dut.B_matrix_flat.value = pack_matrix(b, DATA_BIT_SIZE)

    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    saw_busy = False
    for _ in range(TOTAL_CYCLES + 20):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.busy.value):
            saw_busy = True
        if saw_busy and int(dut.done.value):
            break
    else:
        raise AssertionError("Timed out waiting for done")

    return unpack_matrix(int(dut.Out_matrix_flat.value), ACC_BIT_SIZE)


@logged_test()
async def simple_matrix_test(dut):
    await setup_dut(dut)

    a = build_incremental_matrix(1)
    b = build_incremental_matrix(N * N + 1)

    expected = matmul_expected(a, b)
    actual = await run_matmul(dut, a, b)
    assert actual == expected, "Simple matrix multiply mismatch"


@logged_test()
async def maxed_matrix_test(dut):
    await setup_dut(dut)

    max_val = (1 << DATA_BIT_SIZE) - 1
    a = [[max_val for _ in range(N)] for _ in range(N)]
    b = [[max_val for _ in range(N)] for _ in range(N)]

    expected = matmul_expected(a, b)
    actual = await run_matmul(dut, a, b)
    assert actual == expected, "Max-value matrix multiply mismatch"

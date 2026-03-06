"""Robust cocotb testbench for uart_rx with cycle-accurate golden model."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

CLKS_PER_BIT = 104


class UartRxModel:
    IDLE, START, DATA, STOP = 0, 1, 2, 3

    def __init__(self, clks_per_bit=CLKS_PER_BIT):
        self.clks_per_bit = clks_per_bit
        self.reset()

    def reset(self):
        self.state = self.IDLE
        self.clk_cnt = 0
        self.bit_idx = 0
        self.rx_data = 0
        self.data = 0
        self.valid = 0
        self.rx_sync = 1
        self.rx_d = 1

    def step(self, rst, rx_pin):
        if rst:
            self.reset()
            return self.data, self.valid

        prev_rx_sync = self.rx_sync
        self.rx_sync = rx_pin
        self.rx_d = prev_rx_sync
        self.valid = 0

        if self.state == self.IDLE:
            self.clk_cnt = 0
            self.bit_idx = 0
            if self.rx_d == 0:
                self.state = self.START

        elif self.state == self.START:
            if self.clk_cnt == (self.clks_per_bit - 1) // 2:
                if self.rx_d == 0:
                    self.clk_cnt = 0
                    self.state = self.DATA
                else:
                    self.state = self.IDLE
            else:
                self.clk_cnt += 1

        elif self.state == self.DATA:
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                if self.rx_d:
                    self.rx_data |= (1 << self.bit_idx)
                else:
                    self.rx_data &= ~(1 << self.bit_idx)
                if self.bit_idx == 7:
                    self.bit_idx = 0
                    self.state = self.STOP
                else:
                    self.bit_idx += 1
            else:
                self.clk_cnt += 1

        elif self.state == self.STOP:
            if self.clk_cnt == self.clks_per_bit - 1:
                self.clk_cnt = 0
                self.state = self.IDLE
                if self.rx_d == 1:
                    self.data = self.rx_data
                    self.valid = 1
            else:
                self.clk_cnt += 1

        return self.data, self.valid


async def send_uart_byte(dut, byte_val):
    dut.rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.rx.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.data.value) == 0
    assert int(dut.valid.value) == 0


@cocotb.test()
async def test_single_byte(dut):
    await setup(dut)
    await send_uart_byte(dut, 0xA5)

    seen = False
    for _ in range(CLKS_PER_BIT * 3):
        await RisingEdge(dut.clk)
        if int(dut.valid.value):
            seen = True
            assert int(dut.data.value) == 0xA5
            break
    assert seen, "No valid pulse observed"


@cocotb.test()
async def test_framing_error_rejected(dut):
    await setup(dut)

    dut.rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (0x5A >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 0  # bad stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 1

    for _ in range(CLKS_PER_BIT * 4):
        await RisingEdge(dut.clk)
        assert int(dut.valid.value) == 0, "Framing-error byte should be dropped"


@cocotb.test()
async def test_all_byte_values(dut):
    await setup(dut)
    for val in range(256):
        await send_uart_byte(dut, val)
        seen = False
        for _ in range(CLKS_PER_BIT * 4):
            await RisingEdge(dut.clk)
            if int(dut.valid.value):
                assert int(dut.data.value) == val, f"Expected 0x{val:02X}, got 0x{int(dut.data.value):02X}"
                seen = True
                break
        assert seen, f"No valid for byte 0x{val:02X}"


@cocotb.test()
async def test_randomized_golden_waveform(dut):
    await setup(dut)
    model = UartRxModel(CLKS_PER_BIT)
    rng = random.Random(0x9127)

    # Build a random waveform with valid frames, false starts, and idle regions.
    waveform = [1] * (CLKS_PER_BIT * 6)
    expected_bytes = []

    for _ in range(40):
        mode = rng.choice(["byte", "byte", "false_start", "idle"])
        if mode == "byte":
            b = rng.randint(0, 255)
            expected_bytes.append(b)
            waveform += [0] * CLKS_PER_BIT
            for i in range(8):
                bit = (b >> i) & 1
                waveform += [bit] * CLKS_PER_BIT
            waveform += [1] * CLKS_PER_BIT
            waveform += [1] * rng.randint(0, CLKS_PER_BIT * 3)
        elif mode == "false_start":
            waveform += [0] * rng.randint(1, max(1, (CLKS_PER_BIT // 2) - 2))
            waveform += [1] * rng.randint(1, CLKS_PER_BIT)
        else:
            waveform += [1] * rng.randint(1, CLKS_PER_BIT * 4)

    observed = []
    model_observed = []

    for bit in waveform:
        dut.rx.value = bit
        await RisingEdge(dut.clk)
        m_data, m_valid = model.step(int(dut.rst.value), bit)

        if m_valid:
            model_observed.append(m_data)
        if int(dut.valid.value):
            observed.append(int(dut.data.value))

    assert observed == model_observed, f"DUT bytes {observed} != model {model_observed}"
    assert observed == expected_bytes, f"Decoded bytes {observed} != expected {expected_bytes}"

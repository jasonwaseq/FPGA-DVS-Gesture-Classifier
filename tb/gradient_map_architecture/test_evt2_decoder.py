"""Unit testbench for evt2_decoder with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

GRID_BITS = 4


class Evt2DecoderModel:
    EVT_CD_OFF = 0x0
    EVT_CD_ON = 0x1
    EVT_TIME_HIGH = 0x8

    def __init__(self):
        self.reset()

    def reset(self):
        self.time_high_reg = 0
        self.x_out = 0
        self.y_out = 0
        self.polarity = 0
        self.timestamp = 0
        self.event_valid = 0

    def step(self, data_in, data_valid):
        self.event_valid = 0
        if data_valid:
            pkt_type = (data_in >> 28) & 0xF
            ts_lsb = (data_in >> 22) & 0x3F
            x_raw = (data_in >> 11) & 0x7FF
            y_raw = data_in & 0x7FF

            x_grid = min((x_raw >> 4) & 0x1F, 15)
            y_grid = min((y_raw >> 4) & 0x1F, 15)
            full_ts = ((self.time_high_reg & 0x3FF) << 6) | ts_lsb

            if pkt_type == self.EVT_TIME_HIGH:
                self.time_high_reg = data_in & 0x0FFFFFFF
            elif pkt_type in (self.EVT_CD_OFF, self.EVT_CD_ON):
                self.x_out = x_grid
                self.y_out = y_grid
                self.polarity = 1 if pkt_type == self.EVT_CD_ON else 0
                self.timestamp = full_ts
                self.event_valid = 1
        return self.x_out, self.y_out, self.polarity, self.timestamp, self.event_valid


def build_cd(pkt_type, x, y, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
           ((x & 0x7FF) << 11) | (y & 0x7FF)


def build_time_high(payload):
    return (0x8 << 28) | (payload & 0x0FFFFFFF)


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.data_in.value = 0
    dut.data_valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive(dut, word):
    dut.data_in.value = word
    dut.data_valid.value = 1
    await RisingEdge(dut.clk)
    dut.data_valid.value = 0


@cocotb.test()
async def test_reset(dut):
    await setup(dut)
    assert int(dut.event_valid.value) == 0


@cocotb.test()
async def test_cd_on(dut):
    await setup(dut)
    word = build_cd(0x1, 160, 240, 10)
    await drive(dut, word)
    await RisingEdge(dut.clk)
    assert int(dut.event_valid.value) == 1
    assert int(dut.polarity.value) == 1
    assert int(dut.x_out.value) == min((160 >> 4) & 0x1F, 15)
    assert int(dut.y_out.value) == min((240 >> 4) & 0x1F, 15)


@cocotb.test()
async def test_cd_off(dut):
    await setup(dut)
    word = build_cd(0x0, 80, 80, 5)
    await drive(dut, word)
    await RisingEdge(dut.clk)
    assert int(dut.event_valid.value) == 1
    assert int(dut.polarity.value) == 0


@cocotb.test()
async def test_time_high(dut):
    await setup(dut)
    model = Evt2DecoderModel()
    th = build_time_high(0xAB)
    await drive(dut, th)
    model.step(th, 1)
    await RisingEdge(dut.clk)
    model.step(0, 0)

    cd = build_cd(0x1, 128, 128, 0x15)
    await drive(dut, cd)
    _, _, _, mts, _ = model.step(cd, 1)
    await RisingEdge(dut.clk)
    assert int(dut.timestamp.value) == mts


@cocotb.test()
async def test_clamping(dut):
    await setup(dut)
    word = build_cd(0x1, 0x7FF, 0x7FF, 0)
    await drive(dut, word)
    await RisingEdge(dut.clk)
    assert int(dut.x_out.value) == 15
    assert int(dut.y_out.value) == 15


@cocotb.test()
async def test_unknown_types(dut):
    await setup(dut)
    for pt in [0x2, 0x3, 0x5, 0x9, 0xF]:
        word = (pt << 28) | random.randint(0, 0x0FFFFFFF)
        await drive(dut, word)
        await RisingEdge(dut.clk)
        assert int(dut.event_valid.value) == 0


@cocotb.test()
async def test_golden_random(dut):
    await setup(dut)
    model = Evt2DecoderModel()
    mismatches = 0
    for _ in range(500):
        pt = random.choice([0x0, 0x1, 0x8, 0x4, 0xF])
        if pt in (0x0, 0x1):
            word = build_cd(pt, random.randint(0, 320), random.randint(0, 320),
                            random.randint(0, 63))
        elif pt == 0x8:
            word = build_time_high(random.randint(0, 0x0FFFFFFF))
        else:
            word = (pt << 28) | random.randint(0, 0x0FFFFFFF)

        dv = random.choice([0, 1, 1, 1])
        dut.data_in.value = word
        dut.data_valid.value = dv
        await RisingEdge(dut.clk)
        mx, my, mp, mts, mv = model.step(word, dv)
        await RisingEdge(dut.clk)
        dut.data_valid.value = 0
        model.step(0, 0)

        if mv:
            if int(dut.x_out.value) != mx:
                mismatches += 1
            if int(dut.y_out.value) != my:
                mismatches += 1
            if int(dut.polarity.value) != mp:
                mismatches += 1

    assert mismatches == 0, f"{mismatches} field mismatches"

"""Robust cocotb testbench for evt2_decoder with cycle-accurate golden model."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

GRID_BITS = 4
GRID_SIZE = 1 << GRID_BITS
SENSOR_WIDTH = 320
SENSOR_HEIGHT = 320
REQUIRE_TIME_HIGH = 1

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8


def build_evt2_cd(pkt_type, x, y, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | ((x & 0x7FF) << 11) | (y & 0x7FF)


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


class Evt2DecoderModel:
    def __init__(self):
        self.reset()

    def reset(self):
        self.time_high_reg = 0
        self.have_time_high = 0
        self.x_out = 0
        self.y_out = 0
        self.polarity = 0
        self.timestamp = 0
        self.event_valid = 0

    @staticmethod
    def _grid_map(x_raw, y_raw):
        x_clamped = x_raw if x_raw < SENSOR_WIDTH else SENSOR_WIDTH - 1
        y_clamped = y_raw if y_raw < SENSOR_HEIGHT else SENSOR_HEIGHT - 1
        x_grid = x_clamped // (SENSOR_WIDTH // GRID_SIZE)
        y_grid = y_clamped // (SENSOR_HEIGHT // GRID_SIZE)
        x_grid = min(x_grid, GRID_SIZE - 1)
        y_grid = min(y_grid, GRID_SIZE - 1)
        return x_grid, y_grid

    def step(self, rst, data_in, data_valid, event_ready_i):
        pkt_type = (data_in >> 28) & 0xF
        ts_lsb = (data_in >> 22) & 0x3F
        x_raw = (data_in >> 11) & 0x7FF
        y_raw = data_in & 0x7FF
        time_high_payload = data_in & 0x0FFFFFFF

        is_cd = pkt_type in (EVT_CD_OFF, EVT_CD_ON)
        data_ready = int((not is_cd) or event_ready_i)

        if rst:
            self.reset()
            return data_ready

        self.event_valid = 0
        if data_valid and data_ready:
            if pkt_type == EVT_TIME_HIGH:
                self.time_high_reg = time_high_payload
                self.have_time_high = 1
            elif pkt_type in (EVT_CD_OFF, EVT_CD_ON):
                if (not REQUIRE_TIME_HIGH) or self.have_time_high:
                    self.x_out, self.y_out = self._grid_map(x_raw, y_raw)
                    self.polarity = 1 if pkt_type == EVT_CD_ON else 0
                    self.timestamp = ((self.time_high_reg & 0x0FFFFFFF) << 6) | ts_lsb
                    self.event_valid = 1

        return data_ready


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.data_in.value = 0
    dut.data_valid.value = 0
    dut.event_ready_i.value = 1
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive_and_check(dut, model, rst, data_in, data_valid, event_ready_i, tag):
    dut.rst.value = rst
    dut.data_in.value = data_in
    dut.data_valid.value = data_valid
    dut.event_ready_i.value = event_ready_i

    exp_data_ready = model.step(rst, data_in, data_valid, event_ready_i)

    await RisingEdge(dut.clk)
    await ReadOnly()

    assert int(dut.data_ready.value) == exp_data_ready, \
        f"{tag}: data_ready DUT={int(dut.data_ready.value)} model={exp_data_ready}"
    assert int(dut.x_out.value) == model.x_out, \
        f"{tag}: x_out DUT={int(dut.x_out.value)} model={model.x_out}"
    assert int(dut.y_out.value) == model.y_out, \
        f"{tag}: y_out DUT={int(dut.y_out.value)} model={model.y_out}"
    assert int(dut.polarity.value) == model.polarity, \
        f"{tag}: polarity DUT={int(dut.polarity.value)} model={model.polarity}"
    assert int(dut.timestamp.value) == model.timestamp, \
        f"{tag}: timestamp DUT={int(dut.timestamp.value)} model={model.timestamp}"
    assert int(dut.event_valid.value) == model.event_valid, \
        f"{tag}: event_valid DUT={int(dut.event_valid.value)} model={model.event_valid}"

    await NextTimeStep()


@cocotb.test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.event_valid.value) == 0
    assert int(dut.x_out.value) == 0
    assert int(dut.y_out.value) == 0


@cocotb.test()
async def test_cd_requires_time_high(dut):
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    cd_word = build_evt2_cd(EVT_CD_ON, 100, 120, 7)
    await drive_and_check(dut, model, 0, cd_word, 1, 1, "cd-before-th")
    assert int(dut.event_valid.value) == 0

    th = build_evt2_time_high(0x12345)
    await drive_and_check(dut, model, 0, th, 1, 1, "th")
    await drive_and_check(dut, model, 0, cd_word, 1, 1, "cd-after-th")
    assert int(dut.event_valid.value) == 1


@cocotb.test()
async def test_backpressure_on_cd_only(dut):
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    th = build_evt2_time_high(0x77)
    await drive_and_check(dut, model, 0, th, 1, 0, "th-ready-low")
    assert int(dut.data_ready.value) == 1, "TIME_HIGH should ignore event_ready_i"

    cd = build_evt2_cd(EVT_CD_OFF, 50, 60, 1)
    await drive_and_check(dut, model, 0, cd, 1, 0, "cd-stall")
    assert int(dut.data_ready.value) == 0
    assert int(dut.event_valid.value) == 0

    await drive_and_check(dut, model, 0, cd, 1, 1, "cd-accept")
    assert int(dut.event_valid.value) == 1


@cocotb.test()
async def test_coordinate_clamp_and_timestamp(dut):
    await setup(dut)
    model = Evt2DecoderModel()

    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    await drive_and_check(dut, model, 0, build_evt2_time_high(0x0FFFFFFF), 1, 1, "th")
    word = build_evt2_cd(EVT_CD_ON, 0x7FF, 0x7FF, 0x2A)
    await drive_and_check(dut, model, 0, word, 1, 1, "clamp")

    assert int(dut.x_out.value) == GRID_SIZE - 1
    assert int(dut.y_out.value) == GRID_SIZE - 1
    exp_ts = ((0x0FFFFFFF << 6) | 0x2A)
    assert int(dut.timestamp.value) == exp_ts


@cocotb.test()
async def test_randomized_golden_scoreboard(dut):
    await setup(dut)
    model = Evt2DecoderModel()
    rng = random.Random(0xE172D2)

    # Mirror setup timing in model.
    for _ in range(5):
        await drive_and_check(dut, model, 1, 0, 0, 1, "rst")
    for _ in range(2):
        await drive_and_check(dut, model, 0, 0, 0, 1, "idle")

    for cycle in range(1800):
        pkt_type = rng.choice([0x0, 0x1, 0x2, 0x5, 0x8, 0xF])
        if pkt_type in (EVT_CD_OFF, EVT_CD_ON):
            word = build_evt2_cd(pkt_type, rng.randint(0, 0x7FF), rng.randint(0, 0x7FF), rng.randint(0, 63))
        elif pkt_type == EVT_TIME_HIGH:
            word = build_evt2_time_high(rng.randint(0, 0x0FFFFFFF))
        else:
            word = (pkt_type << 28) | rng.randint(0, 0x0FFFFFFF)

        dv = rng.choice([0, 1, 1, 1])
        ready = rng.choice([0, 1, 1])
        await drive_and_check(dut, model, 0, word, dv, ready, f"rnd-{cycle}")

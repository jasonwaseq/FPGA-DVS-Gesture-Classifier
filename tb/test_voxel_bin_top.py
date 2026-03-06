"""Robust cocotb testbench for voxel_bin_top UART protocol and packetization."""

from collections import deque
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

ST_ACCUM = 0
READOUT_BINS = 8
WINDOW_MS = 400
BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g):
    return (g & 0xF) * 20


def map_internal_to_uart(g_internal):
    if g_internal == 0:
        return 1
    if g_internal == 1:
        return 2
    if g_internal == 2:
        return 3
    return 0


async def wait_for_por_release(dut):
    stable = 0
    for _ in range(5000):
        await RisingEdge(dut.clk)
        if int(dut.rst.value) == 0:
            stable += 1
            if stable >= 6:
                return
        else:
            stable = 0
    raise AssertionError("Timed out waiting for POR deassertion")


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 8)
    await wait_for_por_release(dut)
    await ClockCycles(dut.clk, 4)


async def uart_drive_byte(dut, byte_val):
    # start
    dut.uart_rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)

    # data LSB-first
    for i in range(8):
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)

    # stop
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)


async def uart_receive_byte(dut, timeout_cycles=50000):
    prev = int(dut.uart_tx.value)
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        cur = int(dut.uart_tx.value)
        if prev == 1 and cur == 0:
            break
        prev = cur
    else:
        return None

    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    if int(dut.uart_tx.value) != 0:
        return None

    val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if int(dut.uart_tx.value):
            val |= 1 << i

    await ClockCycles(dut.clk, CLKS_PER_BIT)
    if int(dut.uart_tx.value) != 1:
        return None

    return val


async def send_evt2_word_uart(dut, word):
    b0 = (word >> 24) & 0xFF
    assert b0 not in (0xFC, 0xFD, 0xFE, 0xFF), f"Word starts with command byte 0x{b0:02X}"
    for shift in (24, 16, 8, 0):
        await uart_drive_byte(dut, (word >> shift) & 0xFF)


async def collect_core_words(dut, cycles):
    words = []
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        if int(dut.core_evt_valid.value) and int(dut.core_evt_ready.value):
            words.append(int(dut.core_evt_word.value))
    return words


async def force_core_bin_rollover(dut):
    while int(dut.u_core.u_voxel_binning.state.value) != ST_ACCUM:
        await RisingEdge(dut.clk)
    dut.u_core.u_voxel_binning.timer_ctr.value = CYCLES_PER_BIN_SAFE - 1
    await RisingEdge(dut.clk)


def region_points(name):
    if name == "top":
        ys, xs = range(1, 5), range(4, 12)
    elif name == "bottom":
        ys, xs = range(11, 15), range(4, 12)
    elif name == "left":
        ys, xs = range(4, 12), range(1, 5)
    elif name == "right":
        ys, xs = range(4, 12), range(11, 15)
    else:
        raise ValueError(name)

    pts = []
    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts


@cocotb.test()
async def test_uart_commands_and_word_assembly(dut):
    await setup(dut)

    # Echo command.
    await uart_drive_byte(dut, 0xFF)
    b = await uart_receive_byte(dut)
    assert b == 0x55, f"Echo response mismatch: {b}"

    # Config command.
    await uart_drive_byte(dut, 0xFD)
    c0 = await uart_receive_byte(dut)
    c1 = await uart_receive_byte(dut)
    assert c0 == 0x08 and c1 == 0x08, f"Config response mismatch: {[c0, c1]}"

    # Status command.
    await uart_drive_byte(dut, 0xFE)
    s = await uart_receive_byte(dut)
    assert s is not None, "No status response"
    assert (s & 0xF0) == 0xB0, f"Status high nibble mismatch: 0x{s:02X}"
    assert (s & 0x01) == 0, f"Status bit0 should be 0: 0x{s:02X}"

    # Soft reset command should assert internal rst pulse.
    await uart_drive_byte(dut, 0xFC)
    saw_rst = False
    for _ in range(2000):
        await RisingEdge(dut.clk)
        if int(dut.rst.value):
            saw_rst = True
            break
    assert saw_rst, "Soft reset command did not assert rst"

    # Word assembly path: send one EVT2 word and verify it reaches core word stream.
    word = build_evt2_time_high(0x123456)
    collector = cocotb.start_soon(collect_core_words(dut, cycles=50000))
    await send_evt2_word_uart(dut, word)
    words = await collector
    assert word in words, f"Assembled word 0x{word:08X} not observed on core_evt_word"


@cocotb.test()
async def test_gesture_uart_packet_stream_matches_core(dut):
    await setup(dut)

    expected_bytes = deque()
    observed_bytes = []
    unexpected_bytes = []
    core_gesture_count = 0

    stop = {"flag": False}

    async def core_gesture_monitor():
        nonlocal core_gesture_count
        while not stop["flag"]:
            await RisingEdge(dut.clk)
            if int(dut.rst.value):
                continue
            if int(dut.u_core.gesture_valid.value):
                core_gesture_count += 1
                g_int = int(dut.u_core.gesture.value)
                g_uart = map_internal_to_uart(g_int)
                conf = int(dut.u_core.gesture_confidence.value) & 0xF
                evthi = (int(dut.u_core.debug_event_count.value) >> 4) & 0xF
                expected_bytes.append(0xA0 | g_uart)
                expected_bytes.append((conf << 4) | evthi)

    async def uart_tx_monitor():
        while not stop["flag"]:
            b = await uart_receive_byte(dut, timeout_cycles=20000)
            if b is None:
                continue
            observed_bytes.append(b)
            if expected_bytes:
                exp = expected_bytes.popleft()
                assert b == exp, f"UART packet byte mismatch DUT=0x{b:02X} model=0x{exp:02X}"
            else:
                unexpected_bytes.append(b)

    mon_core = cocotb.start_soon(core_gesture_monitor())
    mon_uart = cocotb.start_soon(uart_tx_monitor())

    rng = random.Random(0xA91B57)

    await send_evt2_word_uart(dut, build_evt2_time_high(0x12345))

    script = [
        "bottom", "bottom", "top", "top",
        "right", "right", "left", "left",
        "bottom", "bottom", "top", "top",
    ]

    for region in script:
        pts = region_points(region)
        for i in range(10):
            gx, gy = rng.choice(pts)
            x_s = sensor_from_grid(gx)
            y_s = sensor_from_grid(gy)
            pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
            word = build_evt2_cd(pkt, x_s, y_s, i & 0x3F)
            await send_evt2_word_uart(dut, word)
        await force_core_bin_rollover(dut)

    # Give pipeline time to flush results and TX queue.
    await ClockCycles(dut.clk, 200000)

    stop["flag"] = True
    await ClockCycles(dut.clk, CLKS_PER_BIT * 20)

    # Drain monitor tasks (best effort).
    mon_core.kill()
    mon_uart.kill()

    assert core_gesture_count > 0, "No core gesture_valid pulses observed"
    assert not expected_bytes, f"Missing UART bytes for queued packets: {list(expected_bytes)}"
    assert not unexpected_bytes, f"Unexpected UART bytes observed: {unexpected_bytes}"

"""Robust cocotb testbench for voxel_bin_top UART protocol and packetization."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE
DRIVE_CLKS_PER_BIT = CLKS_PER_BIT + 2

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

ST_ACCUM = 0
GRID_SIZE = 8
READOUT_BINS = 4
WINDOW_MS = 400
BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS
SENSOR_DIM = 320
BIN_DIV = SENSOR_DIM // GRID_SIZE


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g):
    g = max(0, min(GRID_SIZE - 1, int(g)))
    return min(SENSOR_DIM - 1, (g * BIN_DIV) + (BIN_DIV // 2))


def map_internal_to_uart(g_internal):
    return g_internal & 0x3


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
    await ClockCycles(dut.clk, CLKS_PER_BIT * 3)
    await drain_tx_bytes(dut)


async def uart_drive_byte(dut, byte_val):
    await RisingEdge(dut.clk)
    # start
    dut.uart_rx.value = 0
    await ClockCycles(dut.clk, DRIVE_CLKS_PER_BIT)

    # data LSB-first
    for i in range(8):
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, DRIVE_CLKS_PER_BIT)

    # stop
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, DRIVE_CLKS_PER_BIT)


async def await_rx_byte(dut, timeout_cycles=50000):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.rx_byte_valid.value):
            return int(dut.rx_byte.value)
    return None


async def await_tx_bytes(dut, count, timeout_cycles=200000):
    out = []
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.tx_byte_valid.value):
            out.append(int(dut.tx_byte.value))
            if len(out) >= count:
                return out
    raise AssertionError(f"Timed out waiting for {count} tx byte(s), got {out}")


async def drain_tx_bytes(dut, idle_cycles=CLKS_PER_BIT * 12):
    # Consume any startup bytes if present; return once the stream stays quiet.
    quiet = 0
    while quiet < idle_cycles:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.tx_byte_valid.value):
            quiet = 0
        else:
            quiet += 1


async def await_tx_idle(dut, stable_cycles=CLKS_PER_BIT * 3, timeout_cycles=500000):
    # Wait until TX producer/consumer path has no pending activity for a stable window.
    quiet = 0
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()
        busy = int(dut.tx_busy.value)
        out_valid = int(dut.tx_fifo_out_valid.value)
        in_valid = int(dut.tx_fifo_in_valid.value)
        second_pending = int(dut.second_byte_pending.value)
        if busy or out_valid or in_valid or second_pending:
            quiet = 0
        else:
            quiet += 1
            if quiet >= stable_cycles:
                return
    raise AssertionError("Timed out waiting for TX path to become idle")


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
    x_lo = max(0, GRID_SIZE // 8)
    x_hi = min(GRID_SIZE, GRID_SIZE - (GRID_SIZE // 8))
    y_lo = x_lo
    y_hi = x_hi
    band = max(2, GRID_SIZE // 4)

    if name == "top":
        ys, xs = range(y_lo, min(y_lo + band, GRID_SIZE)), range(x_lo, x_hi)
    elif name == "bottom":
        ys, xs = range(max(GRID_SIZE - band, 0), y_hi), range(x_lo, x_hi)
    elif name == "left":
        ys, xs = range(y_lo, y_hi), range(x_lo, min(x_lo + band, GRID_SIZE))
    elif name == "right":
        ys, xs = range(y_lo, y_hi), range(max(GRID_SIZE - band, 0), x_hi)
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
    # await_rx_byte must start concurrently with uart_drive_byte: rx_byte_valid is a
    # 1-cycle pulse that fires during the STOP-bit sampling phase (~72 clks before
    # uart_drive_byte returns), so a sequential call after drive always misses it.
    await await_tx_idle(dut)
    echo_task  = cocotb.start_soon(await_tx_bytes(dut, 1))
    rx_task    = cocotb.start_soon(await_rx_byte(dut, timeout_cycles=CLKS_PER_BIT * 200))
    await uart_drive_byte(dut, 0xFF)
    rx = await rx_task
    assert rx == 0xFF, f"Echo command decode mismatch: 0x{(rx if rx is not None else -1):02X}"
    b = (await echo_task)[0]
    assert b == 0x55, f"Echo response mismatch: {b}"

    # Config command.
    await await_tx_idle(dut)
    cfg_task = cocotb.start_soon(await_tx_bytes(dut, 2))
    rx_task  = cocotb.start_soon(await_rx_byte(dut, timeout_cycles=CLKS_PER_BIT * 200))
    await uart_drive_byte(dut, 0xFD)
    rx = await rx_task
    assert rx == 0xFD, f"Config command decode mismatch: 0x{(rx if rx is not None else -1):02X}"
    c0, c1 = await cfg_task
    assert c0 == 0x04 and c1 == 0x04, f"Config response mismatch: {[c0, c1]}"

    # Status command.
    await await_tx_idle(dut)
    status_task = cocotb.start_soon(await_tx_bytes(dut, 1))
    rx_task     = cocotb.start_soon(await_rx_byte(dut, timeout_cycles=CLKS_PER_BIT * 200))
    await uart_drive_byte(dut, 0xFE)
    rx = await rx_task
    assert rx == 0xFE, f"Status command decode mismatch: 0x{(rx if rx is not None else -1):02X}"
    s = (await status_task)[0]
    assert (s & 0xF0) == 0xB0, f"Status high nibble mismatch: 0x{s:02X}"
    assert (s & 0x01) == 0, f"Status bit0 should be 0: 0x{s:02X}"

    # Soft reset command should assert internal rst pulse.
    # rst de-asserts only SOFT_RESET_CYCLES (64) clks after rx_byte_valid fires,
    # which is before uart_drive_byte returns — monitor concurrently.
    async def _await_rst(timeout=CLKS_PER_BIT * 200):
        for _ in range(timeout):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.rst.value):
                return True
        return False

    rst_task = cocotb.start_soon(_await_rst())
    await uart_drive_byte(dut, 0xFC)
    saw_rst = await rst_task
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

    produced_bytes = []
    dequeued_bytes = []
    core_gesture_count = 0

    stop = {"flag": False}

    async def core_gesture_monitor():
        nonlocal core_gesture_count
        prev_valid = 0
        while not stop["flag"]:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.rst.value):
                prev_valid = 0
                continue
            if prev_valid:
                core_gesture_count += 1

            prev_valid = int(dut.u_core.gesture_valid.value)

    async def uart_tx_monitor():
        while not stop["flag"]:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.tx_fifo_in_valid.value) and int(dut.tx_fifo_in_ready.value):
                produced_bytes.append(int(dut.tx_fifo_in_data.value))
            if int(dut.tx_fifo_out_valid.value) and int(dut.tx_fifo_out_ready.value):
                dequeued_bytes.append(int(dut.tx_fifo_out_data.value))

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
    mon_core.cancel()
    mon_uart.cancel()

    assert core_gesture_count > 0, "No core gesture_valid pulses observed"
    assert len(dequeued_bytes) == (2 * core_gesture_count), \
        (f"Expected 2 UART bytes per core gesture, got {len(dequeued_bytes)} bytes "
         f"for {core_gesture_count} gestures")
    assert dequeued_bytes == produced_bytes, \
        f"UART packet stream mismatch\nTX_DEQ: {dequeued_bytes}\nTX_IN:  {produced_bytes}"

"""Integration cocotb testbench for voxel_bin_core with golden scoreboards."""

from collections import deque
from pathlib import Path
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

GRID_SIZE = 16
NUM_BINS = 8
READOUT_BINS = 8
FEATURE_COUNT = GRID_SIZE * GRID_SIZE * READOUT_BINS
PASS_MARGIN = 64
PERSISTENCE_COUNT = 2
CONF_BITS = 4
CONF_SHIFT = 4

CLK_FREQ_HZ = 12_000_000
WINDOW_MS = 400
BIN_DURATION_MS = WINDOW_MS // READOUT_BINS
CYCLES_PER_BIN_SAFE = (CLK_FREQ_HZ // 1000) * BIN_DURATION_MS

EVT_CD_OFF = 0x0
EVT_CD_ON = 0x1
EVT_TIME_HIGH = 0x8

ST_ACCUM = 0


def build_evt2_time_high(payload):
    return (EVT_TIME_HIGH << 28) | (payload & 0x0FFFFFFF)


def build_evt2_cd(pkt_type, x_sensor, y_sensor, ts_lsb):
    return ((pkt_type & 0xF) << 28) | ((ts_lsb & 0x3F) << 22) | \
        ((x_sensor & 0x7FF) << 11) | (y_sensor & 0x7FF)


def sensor_from_grid(g):
    # Decoder uses x_grid = x_raw / 20 for 320->16 mapping.
    return (g & 0xF) * 20


class Evt2DecoderModel:
    def __init__(self):
        self.time_high = 0
        self.have_time_high = False

    def on_word(self, word):
        pkt = (word >> 28) & 0xF
        ts_lsb = (word >> 22) & 0x3F
        x_raw = (word >> 11) & 0x7FF
        y_raw = word & 0x7FF

        if pkt == EVT_TIME_HIGH:
            self.time_high = word & 0x0FFFFFFF
            self.have_time_high = True
            return None

        if pkt not in (EVT_CD_OFF, EVT_CD_ON):
            return None

        if not self.have_time_high:
            return None

        x_clamped = min(x_raw, 319)
        y_clamped = min(y_raw, 319)
        x_grid = min(x_clamped // 20, 15)
        y_grid = min(y_clamped // 20, 15)
        pol = 1 if pkt == EVT_CD_ON else 0
        ts = (self.time_high << 6) | ts_lsb
        return (x_grid, y_grid, pol, ts)


class GesturePersistenceModel:
    def __init__(self):
        self.last_pass_class = 0
        self.pass_streak = 0
        self.gesture = 0

    @staticmethod
    def _confidence_from_margin(margin):
        if margin <= 0:
            return 0
        c = margin >> CONF_SHIFT
        return min((1 << CONF_BITS) - 1, c)

    def step(self, class_id, class_pass, margin):
        gesture_valid = 0
        conf = 0

        if class_pass:
            if class_id == self.last_pass_class:
                if self.pass_streak < PERSISTENCE_COUNT:
                    next_streak = self.pass_streak + 1
                else:
                    next_streak = self.pass_streak
            else:
                next_streak = 1

            self.last_pass_class = class_id
            self.pass_streak = next_streak

            conf = self._confidence_from_margin(margin)
            if next_streak >= PERSISTENCE_COUNT:
                self.gesture = class_id
                gesture_valid = 1
        else:
            self.pass_streak = 0

        return self.gesture, gesture_valid, conf


class ScoreModel:
    def __init__(self, weights_per_class):
        self.weights = weights_per_class

    @staticmethod
    def _argmax_with_second(vals):
        best_i = 0
        best = vals[0]
        second = -10**30
        for i in range(1, len(vals)):
            if vals[i] > best:
                second = best
                best = vals[i]
                best_i = i
            elif vals[i] > second:
                second = vals[i]
        return best_i, best, second

    def classify(self, features):
        scores = [0, 0, 0, 0]
        for i, feat in enumerate(features):
            f = int(feat)
            for c in range(4):
                scores[c] += f * self.weights[c][i]

        best_class, best, second = self._argmax_with_second(scores)
        margin = best - second
        class_pass = int(margin > PASS_MARGIN)
        return best_class, class_pass, margin


def load_quantized_weights():
    weights_path = Path(__file__).resolve().parents[1] / "8192weights.txt"
    lines = weights_path.read_text(encoding="ascii").splitlines()

    def quantize(line):
        try:
            f = float(line.strip())
        except ValueError:
            return 0
        q = int(f * 1024)  # $rtoi-style trunc toward zero
        if q < 0:
            q = 0
        if q > 255:
            q = 255
        return q

    qvals = [quantize(line) for line in lines]
    expected_len = 4 * FEATURE_COUNT
    if len(qvals) < expected_len:
        qvals.extend([0] * (expected_len - len(qvals)))

    weights = []
    for c in range(4):
        start = c * FEATURE_COUNT
        weights.append(qvals[start:start + FEATURE_COUNT])
    return weights


class CoreHarness:
    def __init__(self, dut):
        self.dut = dut
        self.decoder = Evt2DecoderModel()
        self.persistence = GesturePersistenceModel()
        self.scorer = ScoreModel(load_quantized_weights())

        self.expected_decoded = deque()
        self.current_window = []

        self.expected_gestures = []
        self.observed_gestures = []

        self.accepted_words = 0
        self.completed_windows = 0

    async def setup(self):
        cocotb.start_soon(Clock(self.dut.clk, 10, unit="ns").start())
        self.dut.rst.value = 1
        self.dut.evt_word.value = 0
        self.dut.evt_word_valid.value = 0
        await ClockCycles(self.dut.clk, 8)
        self.dut.rst.value = 0
        await self.tick(4)

    def _sample_cycle(self):
        if int(self.dut.u_evt2_decoder.event_valid.value):
            observed = (
                int(self.dut.u_evt2_decoder.x_out.value),
                int(self.dut.u_evt2_decoder.y_out.value),
                int(self.dut.u_evt2_decoder.polarity.value),
                int(self.dut.u_evt2_decoder.timestamp.value),
            )
            assert self.expected_decoded, f"Unexpected decoded event {observed}"
            expected = self.expected_decoded.popleft()
            assert observed == expected, f"Decoded mismatch DUT={observed} model={expected}"

        if int(self.dut.u_voxel_binning.readout_valid.value):
            idx = int(self.dut.u_voxel_binning.readout_index.value)
            assert idx == len(self.current_window), \
                f"Readout index mismatch DUT={idx}, expected={len(self.current_window)}"
            self.current_window.append(int(self.dut.u_voxel_binning.readout_data.value))

            if int(self.dut.u_voxel_binning.readout_last.value):
                assert len(self.current_window) == FEATURE_COUNT, \
                    f"Feature window length {len(self.current_window)} != {FEATURE_COUNT}"
                best, cls_pass, margin = self.scorer.classify(self.current_window)
                g, gv, gc = self.persistence.step(best, cls_pass, margin)
                if gv:
                    self.expected_gestures.append((g, gc))
                self.current_window = []
                self.completed_windows += 1

        if int(self.dut.gesture_valid.value):
            self.observed_gestures.append((
                int(self.dut.gesture.value),
                int(self.dut.gesture_confidence.value),
            ))

    async def tick(self, cycles=1):
        for _ in range(cycles):
            await RisingEdge(self.dut.clk)
            self._sample_cycle()

    async def send_word(self, word):
        while int(self.dut.evt_word_ready.value) == 0:
            await self.tick(1)

        self.dut.evt_word.value = word
        self.dut.evt_word_valid.value = 1
        await self.tick(1)
        self.dut.evt_word_valid.value = 0

        evt = self.decoder.on_word(word)
        if evt is not None:
            self.expected_decoded.append(evt)
        self.accepted_words += 1

    async def force_bin_rollover(self):
        while int(self.dut.u_voxel_binning.state.value) != ST_ACCUM:
            await self.tick(1)
        self.dut.u_voxel_binning.timer_ctr.value = CYCLES_PER_BIN_SAFE - 1
        await self.tick(1)

    async def wait_quiet(self, quiet_cycles=2000, timeout=200000):
        prev_g = len(self.observed_gestures)
        prev_w = self.completed_windows
        q = 0
        for _ in range(timeout):
            await self.tick(1)
            now_g = len(self.observed_gestures)
            now_w = self.completed_windows
            if now_g == prev_g and now_w == prev_w:
                q += 1
                if q >= quiet_cycles:
                    return
            else:
                q = 0
                prev_g = now_g
                prev_w = now_w
        raise AssertionError("Timeout waiting for pipeline quiet")


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


async def drive_bin_traffic(h, rng, region, events=28):
    pts = region_points(region)
    for i in range(events):
        if i % 10 == 0:
            await h.send_word(build_evt2_time_high(rng.randint(0, 0x0FFFFFFF)))

        gx, gy = rng.choice(pts)
        x_s = sensor_from_grid(gx)
        y_s = sensor_from_grid(gy)
        pkt = EVT_CD_ON if (i & 1) else EVT_CD_OFF
        await h.send_word(build_evt2_cd(pkt, x_s, y_s, i & 0x3F))

        if i % 13 == 0:
            bad = (0xF << 28) | rng.randint(0, 0x0FFFFFFF)
            await h.send_word(bad)


@cocotb.test()
async def test_voxel_bin_core_end_to_end_golden(dut):
    rng = random.Random(0xC011E0)
    h = CoreHarness(dut)
    await h.setup()

    await h.send_word(build_evt2_time_high(0x12345))

    script = [
        "bottom", "bottom", "top", "top",
        "right", "right", "left", "left",
        "bottom", "bottom", "top", "top",
    ]

    for region in script:
        await drive_bin_traffic(h, rng, region, events=30)
        await h.force_bin_rollover()

    await h.wait_quiet()

    assert not h.expected_decoded, f"Unmatched decoded events: {len(h.expected_decoded)}"
    assert not h.current_window, "Partial readout window remained"
    assert h.completed_windows > 0, "No completed readout windows observed"

    assert h.observed_gestures == h.expected_gestures, \
        f"Gesture stream mismatch\nDUT:   {h.observed_gestures}\nMODEL: {h.expected_gestures}"

    # debug_event_count is 8-bit saturating wrap counter of accepted words.
    assert int(dut.debug_event_count.value) == (h.accepted_words & 0xFF), \
        "debug_event_count mismatch"

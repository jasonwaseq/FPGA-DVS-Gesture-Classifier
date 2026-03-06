"""Robust cocotb testbench for gesture_classifier using a cycle-accurate golden model."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, NextTimeStep, ReadOnly, RisingEdge

NUM_CLASSES = 4
SCORE_BITS = 32
PASS_MARGIN = 64
PERSISTENCE_COUNT = 2
CONF_BITS = 8
CONF_SHIFT = 4
CONF_MAX = (1 << CONF_BITS) - 1
SCORE_MASK = (1 << SCORE_BITS) - 1


def to_signed(val, bits):
    val &= (1 << bits) - 1
    if val & (1 << (bits - 1)):
        return val - (1 << bits)
    return val


def from_signed(val, bits):
    return val & ((1 << bits) - 1)


def pack_scores(scores):
    packed = 0
    for i, s in enumerate(scores):
        packed |= from_signed(s, SCORE_BITS) << (i * SCORE_BITS)
    return packed


class GestureClassifierModel:
    """Cycle-accurate golden model for rtl/gesture_classifier.sv defaults."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.class_gesture = 0
        self.class_valid = 0
        self.class_pass = 0
        self.gesture = 0
        self.gesture_valid = 0
        self.gesture_confidence = 0
        self.last_pass_class = 0
        self.pass_streak = 0
        self.debug_state = 0

    def step(self, scores_flat, scores_valid):
        self.class_valid = 0
        self.class_pass = 0
        self.gesture_valid = 0
        self.debug_state = 0

        if not scores_valid:
            return

        scores = [to_signed((scores_flat >> (i * SCORE_BITS)) & SCORE_MASK, SCORE_BITS)
                  for i in range(NUM_CLASSES)]

        max_score = scores[0]
        second_score = -(1 << (SCORE_BITS - 1))
        max_class = 0

        for i in range(1, NUM_CLASSES):
            if scores[i] > max_score:
                second_score = max_score
                max_score = scores[i]
                max_class = i
            elif scores[i] > second_score:
                second_score = scores[i]

        margin = max_score - second_score
        passed = int(margin > PASS_MARGIN)

        self.class_gesture = max_class
        self.class_valid = 1
        self.class_pass = passed

        if passed:
            if max_class == self.last_pass_class:
                if self.pass_streak < PERSISTENCE_COUNT:
                    next_streak = self.pass_streak + 1
                else:
                    next_streak = self.pass_streak
            else:
                next_streak = 1

            self.last_pass_class = max_class
            self.pass_streak = next_streak

            if next_streak >= PERSISTENCE_COUNT:
                self.gesture = max_class
                self.gesture_valid = 1
                self.debug_state = 2
            else:
                self.debug_state = 1

            conf_shifted = margin >> CONF_SHIFT
            if conf_shifted > CONF_MAX:
                self.gesture_confidence = CONF_MAX
            elif margin <= 0:
                self.gesture_confidence = 0
            else:
                self.gesture_confidence = conf_shifted & CONF_MAX
        else:
            self.pass_streak = 0
            self.debug_state = 0


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst.value = 1
    dut.scores_flat.value = 0
    dut.scores_valid.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def drive_and_check(dut, model, scores, valid, tag):
    packed = pack_scores(scores)
    dut.scores_flat.value = packed
    dut.scores_valid.value = valid

    model.step(packed, valid)

    await RisingEdge(dut.clk)
    await ReadOnly()

    assert int(dut.class_gesture.value) == model.class_gesture, \
        f"{tag}: class_gesture DUT={int(dut.class_gesture.value)} model={model.class_gesture}"
    assert int(dut.class_valid.value) == model.class_valid, \
        f"{tag}: class_valid DUT={int(dut.class_valid.value)} model={model.class_valid}"
    assert int(dut.class_pass.value) == model.class_pass, \
        f"{tag}: class_pass DUT={int(dut.class_pass.value)} model={model.class_pass}"
    assert int(dut.gesture.value) == model.gesture, \
        f"{tag}: gesture DUT={int(dut.gesture.value)} model={model.gesture}"
    assert int(dut.gesture_valid.value) == model.gesture_valid, \
        f"{tag}: gesture_valid DUT={int(dut.gesture_valid.value)} model={model.gesture_valid}"
    assert int(dut.gesture_confidence.value) == model.gesture_confidence, \
        f"{tag}: confidence DUT={int(dut.gesture_confidence.value)} model={model.gesture_confidence}"
    assert int(dut.debug_state.value) == model.debug_state, \
        f"{tag}: debug_state DUT={int(dut.debug_state.value)} model={model.debug_state}"

    await NextTimeStep()


@cocotb.test()
async def test_reset_defaults(dut):
    await setup(dut)
    assert int(dut.class_valid.value) == 0
    assert int(dut.class_pass.value) == 0
    assert int(dut.gesture_valid.value) == 0
    assert int(dut.debug_state.value) == 0


@cocotb.test()
async def test_tie_break_and_threshold(dut):
    await setup(dut)
    model = GestureClassifierModel()

    # All tied: class 0 should win tie-break.
    await drive_and_check(dut, model, [10, 10, 10, 10], 1, "tie")
    assert int(dut.class_gesture.value) == 0

    # Margin exactly PASS_MARGIN should fail (strict >).
    await drive_and_check(dut, model, [100, 36, 36, 36], 1, "margin-eq")
    assert int(dut.class_pass.value) == 0

    # Margin PASS_MARGIN+1 should pass.
    await drive_and_check(dut, model, [101, 36, 36, 36], 1, "margin-gt")
    assert int(dut.class_pass.value) == 1


@cocotb.test()
async def test_persistence_and_class_change(dut):
    await setup(dut)
    model = GestureClassifierModel()

    # Two consecutive passes on same class needed to assert gesture_valid.
    await drive_and_check(dut, model, [300, 0, -10, -20], 1, "persist-1")
    assert int(dut.gesture_valid.value) == 0
    await drive_and_check(dut, model, [350, 0, -10, -20], 1, "persist-2")
    assert int(dut.gesture_valid.value) == 1

    # Class change should restart streak.
    await drive_and_check(dut, model, [0, 400, -10, -20], 1, "change-1")
    assert int(dut.gesture_valid.value) == 0
    await drive_and_check(dut, model, [0, 420, -10, -20], 1, "change-2")
    assert int(dut.gesture_valid.value) == 1


@cocotb.test()
async def test_confidence_edges(dut):
    await setup(dut)
    model = GestureClassifierModel()

    # Build streak first.
    await drive_and_check(dut, model, [300, 0, 0, 0], 1, "warm-1")
    await drive_and_check(dut, model, [301, 0, 0, 0], 1, "warm-2")

    # Saturation case.
    huge = (1 << 20)
    await drive_and_check(dut, model, [huge, -huge, -huge, -huge], 1, "conf-sat")
    assert int(dut.gesture_confidence.value) == CONF_MAX

    # Not-valid cycle keeps confidence stable (model enforces hold behavior).
    prev = int(dut.gesture_confidence.value)
    await drive_and_check(dut, model, [0, 0, 0, 0], 0, "conf-hold")
    assert int(dut.gesture_confidence.value) == prev


@cocotb.test()
async def test_randomized_golden_scoreboard(dut):
    await setup(dut)
    model = GestureClassifierModel()
    rng = random.Random(0xBADC0DE)

    for cycle in range(1500):
        valid = rng.choice([0, 1, 1, 1])
        scores = [rng.randint(-(1 << 20), (1 << 20) - 1) for _ in range(NUM_CLASSES)]
        await drive_and_check(dut, model, scores, valid, f"rnd-{cycle}")

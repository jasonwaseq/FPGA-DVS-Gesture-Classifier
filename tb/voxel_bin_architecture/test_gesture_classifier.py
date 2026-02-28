"""Unit testbench for gesture_classifier with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

ACC_SUM_BITS = 18
PERSISTENCE_COUNT = 2


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class VBGestureClassifierModel:
    """Golden model for the persistence-filter gesture classifier."""

    ST_IDLE, ST_TRACKING, ST_CONFIRMED = 0, 1, 2

    def __init__(self, persistence_count=PERSISTENCE_COUNT):
        self.persistence_count = persistence_count
        self.reset()

    def reset(self):
        self.state = self.ST_IDLE
        self.last_gesture = 0
        self.match_count = 0
        self.gesture = 0
        self.gesture_valid = 0
        self.gesture_confidence = 0

    def step(self, class_gesture, class_valid, class_pass, abs_delta_x, abs_delta_y):
        """One clock cycle. Returns (gesture, gesture_valid, gesture_confidence)."""
        self.gesture_valid = 0

        if class_valid:
            if class_pass:
                if class_gesture == self.last_gesture:
                    if self.match_count < self.persistence_count:
                        self.match_count += 1

                    if self.match_count >= self.persistence_count - 1:
                        self.state = self.ST_CONFIRMED
                        self.gesture = class_gesture
                        self.gesture_valid = 1
                        self.match_count = 0

                        if abs_delta_x > abs_delta_y:
                            self.gesture_confidence = 15 if abs_delta_x > 255 \
                                else (abs_delta_x >> 4) & 0xF
                        else:
                            self.gesture_confidence = 15 if abs_delta_y > 255 \
                                else (abs_delta_y >> 4) & 0xF
                    else:
                        self.state = self.ST_TRACKING
                else:
                    self.last_gesture = class_gesture
                    self.match_count = 0
                    self.state = self.ST_TRACKING
            else:
                self.match_count = 0
                self.state = self.ST_IDLE

        return self.gesture, self.gesture_valid, self.gesture_confidence


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.class_gesture.value = 0
    dut.class_valid.value = 0
    dut.class_pass.value = 0
    dut.abs_delta_x.value = 0
    dut.abs_delta_y.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


async def classify(dut, gesture, pass_flag, dx=100, dy=0):
    dut.class_gesture.value = gesture
    dut.class_valid.value = 1
    dut.class_pass.value = pass_flag
    dut.abs_delta_x.value = dx
    dut.abs_delta_y.value = dy
    await RisingEdge(dut.clk)
    dut.class_valid.value = 0
    dut.class_pass.value = 0


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    """All outputs zero after reset."""
    await setup(dut)
    assert int(dut.gesture_valid.value) == 0
    assert int(dut.gesture.value) == 0
    assert int(dut.gesture_confidence.value) == 0


@cocotb.test()
async def test_single_pass_not_enough(dut):
    """A single passing classification should NOT trigger gesture_valid."""
    await setup(dut)
    await classify(dut, 0, 1)
    await RisingEdge(dut.clk)
    if PERSISTENCE_COUNT > 1:
        assert int(dut.gesture_valid.value) == 0


@cocotb.test()
async def test_persistence_triggers(dut):
    """PERSISTENCE_COUNT consecutive matching passes should trigger gesture_valid."""
    await setup(dut)
    for i in range(PERSISTENCE_COUNT):
        await classify(dut, 1, 1, dx=200)
        await RisingEdge(dut.clk)

    found = False
    for _ in range(5):
        if int(dut.gesture_valid.value) == 1:
            found = True
            assert int(dut.gesture.value) == 1
            break
        await RisingEdge(dut.clk)
    assert found, "gesture_valid not asserted after persistence count reached"


@cocotb.test()
async def test_class_change_resets_count(dut):
    """Changing the gesture class should reset the match counter."""
    await setup(dut)
    await classify(dut, 0, 1)
    await RisingEdge(dut.clk)
    await classify(dut, 1, 1)
    await RisingEdge(dut.clk)
    assert int(dut.gesture_valid.value) == 0

    for _ in range(PERSISTENCE_COUNT):
        await classify(dut, 1, 1, dx=200)
        await RisingEdge(dut.clk)

    found = False
    for _ in range(5):
        if int(dut.gesture_valid.value) == 1:
            found = True
            break
        await RisingEdge(dut.clk)
    assert found


@cocotb.test()
async def test_no_pass_resets(dut):
    """class_pass=0 should reset match count to 0."""
    await setup(dut)
    await classify(dut, 0, 1)
    await RisingEdge(dut.clk)
    await classify(dut, 0, 0)  # fail
    await RisingEdge(dut.clk)
    await classify(dut, 0, 1)
    await RisingEdge(dut.clk)
    assert int(dut.gesture_valid.value) == 0


@cocotb.test()
async def test_confidence_calculation(dut):
    """Confidence should be (dominant_magnitude >> 4), capped at 15."""
    await setup(dut)
    for _ in range(PERSISTENCE_COUNT):
        await classify(dut, 2, 1, dx=0, dy=300)
        await RisingEdge(dut.clk)

    for _ in range(5):
        if int(dut.gesture_valid.value) == 1:
            conf = int(dut.gesture_confidence.value)
            assert conf == 15, f"Expected confidence 15 for dy=300, got {conf}"
            break
        await RisingEdge(dut.clk)

    await ClockCycles(dut.clk, 5)

    for _ in range(PERSISTENCE_COUNT):
        await classify(dut, 3, 1, dx=80, dy=0)
        await RisingEdge(dut.clk)

    for _ in range(5):
        if int(dut.gesture_valid.value) == 1:
            conf = int(dut.gesture_confidence.value)
            expected = (80 >> 4) & 0xF
            assert conf == expected, f"Expected confidence {expected} for dx=80, got {conf}"
            break
        await RisingEdge(dut.clk)


@cocotb.test()
async def test_golden_model_random(dut):
    """Random classification sequence, compare DUT vs golden model."""
    await setup(dut)
    model = VBGestureClassifierModel(PERSISTENCE_COUNT)

    dut_gestures = []
    model_gestures = []

    for _ in range(100):
        gesture = random.randint(0, 3)
        pass_flag = random.choice([0, 1, 1, 1])
        dx = random.randint(0, 500)
        dy = random.randint(0, 500)

        await classify(dut, gesture, pass_flag, dx, dy)
        mg, mv, mc = model.step(gesture, 1, pass_flag, dx, dy)
        await RisingEdge(dut.clk)

        dut_valid = int(dut.gesture_valid.value)
        dut_gesture = int(dut.gesture.value)

        if mv:
            model_gestures.append(mg)
        if dut_valid:
            dut_gestures.append(dut_gesture)

    assert dut_gestures == model_gestures, \
        f"Gesture sequences differ: DUT={dut_gestures}, model={model_gestures}"

"""Unit testbench for systolic_array with golden reference model."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import random

NUM_CLASSES = 4
NUM_CELLS = 16
VALUE_BITS = 8
WEIGHT_BITS = 8
ACC_BITS = 24


# ---------------------------------------------------------------------------
# Golden reference model
# ---------------------------------------------------------------------------
class GMSystolicModel:
    """Golden model for the sequential (1 cell/cycle) systolic array."""

    def __init__(self, num_classes=NUM_CLASSES, num_cells=NUM_CELLS,
                 value_bits=VALUE_BITS, weight_bits=WEIGHT_BITS, acc_bits=ACC_BITS):
        self.num_classes = num_classes
        self.num_cells = num_cells
        self.value_bits = value_bits
        self.weight_bits = weight_bits
        self.acc_bits = acc_bits

    def to_signed(self, val, bits):
        val = val & ((1 << bits) - 1)
        return val - (1 << bits) if val >= (1 << (bits - 1)) else val

    def compute(self, features, weights):
        """
        features: list of NUM_CELLS unsigned values
        weights: 2D list [cell][class] of unsigned WEIGHT_BITS values (interpreted signed)
        Returns (best_class, scores).
        """
        scores = [0] * self.num_classes
        for i in range(self.num_cells):
            feat = features[i] & ((1 << self.value_bits) - 1)
            for k in range(self.num_classes):
                w = self.to_signed(weights[i][k], self.weight_bits)
                scores[k] += feat * w

        for k in range(self.num_classes):
            scores[k] = self.to_signed(scores[k], self.acc_bits)

        best = 0
        for k in range(1, self.num_classes):
            if scores[k] > scores[best]:
                best = k
        return best, scores


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst.value = 1
    dut.start.value = 0
    dut.feature_in.value = 0
    dut.w_data_flat.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst.value = 0
    await ClockCycles(dut.clk, 2)


def pack_weights_for_addr(weight_matrix, addr):
    """Pack NUM_CLASSES weights for a given address into w_data_flat."""
    result = 0
    for k in range(NUM_CLASSES):
        w = weight_matrix[addr][k] & ((1 << WEIGHT_BITS) - 1)
        result |= w << (k * WEIGHT_BITS)
    return result


async def run_inference(dut, features, weights):
    """Start the systolic array, feed features, and wait for result."""
    dut.start.value = 1
    dut.feature_in.value = features[0] & ((1 << VALUE_BITS) - 1)
    await RisingEdge(dut.clk)
    dut.start.value = 0

    for i in range(NUM_CELLS):
        dut.feature_in.value = features[i] & ((1 << VALUE_BITS) - 1)
        w_addr = int(dut.w_addr.value) if hasattr(dut, 'w_addr') else i
        w_addr = min(w_addr, NUM_CELLS - 1)
        dut.w_data_flat.value = pack_weights_for_addr(weights, w_addr)
        await RisingEdge(dut.clk)

    for _ in range(10):
        await RisingEdge(dut.clk)
        if int(dut.result_valid.value) == 1:
            return int(dut.best_class.value)

    return None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_reset(dut):
    await setup(dut)
    assert int(dut.result_valid.value) == 0


@cocotb.test()
async def test_all_zeros(dut):
    """Zero features should produce zero scores."""
    await setup(dut)
    features = [0] * NUM_CELLS
    weights = [[0] * NUM_CLASSES for _ in range(NUM_CELLS)]
    result = await run_inference(dut, features, weights)
    assert result is not None, "No result_valid"


@cocotb.test()
async def test_uniform_features(dut):
    """Uniform features with known weights should produce predictable argmax."""
    await setup(dut)
    model = GMSystolicModel()

    features = [10] * NUM_CELLS
    weights = [[0] * NUM_CLASSES for _ in range(NUM_CELLS)]
    for i in range(NUM_CELLS):
        weights[i][2] = 5  # class 2 gets positive weight

    expected_class, _ = model.compute(features, weights)
    dut_class = await run_inference(dut, features, weights)
    assert dut_class == expected_class, f"DUT={dut_class}, model={expected_class}"


@cocotb.test()
async def test_golden_random(dut):
    """Randomized features/weights, compare argmax vs golden model."""
    await setup(dut)
    model = GMSystolicModel()

    for trial in range(5):
        features = [random.randint(0, (1 << VALUE_BITS) - 1) for _ in range(NUM_CELLS)]
        weights = [[random.randint(0, 255) for _ in range(NUM_CLASSES)]
                    for _ in range(NUM_CELLS)]

        expected_class, _ = model.compute(features, weights)

        dut.start.value = 1
        dut.feature_in.value = features[0]
        await RisingEdge(dut.clk)
        dut.start.value = 0

        for i in range(NUM_CELLS):
            dut.feature_in.value = features[i]
            addr = int(dut.w_addr.value)
            addr = min(addr, NUM_CELLS - 1)
            dut.w_data_flat.value = pack_weights_for_addr(weights, addr)
            await RisingEdge(dut.clk)

        found = False
        for _ in range(10):
            await RisingEdge(dut.clk)
            if int(dut.result_valid.value) == 1:
                dut_class = int(dut.best_class.value)
                assert dut_class == expected_class, \
                    f"Trial {trial}: DUT={dut_class}, model={expected_class}"
                found = True
                break

        assert found, f"Trial {trial}: no result_valid"
        await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_negative_weights_dominant(dut):
    """Class with most negative dot product should not win."""
    await setup(dut)
    model = GMSystolicModel()

    features = [100] * NUM_CELLS
    weights = [[0] * NUM_CLASSES for _ in range(NUM_CELLS)]
    for i in range(NUM_CELLS):
        weights[i][0] = 0xF0  # -16 signed
        weights[i][1] = 0x10  # +16 signed

    expected_class, expected_scores = model.compute(features, weights)
    assert expected_class == 1, f"Model should pick class 1, got {expected_class}"

    dut_class = await run_inference(dut, features, weights)
    assert dut_class == 1, f"DUT should pick class 1, got {dut_class}"


@cocotb.test()
async def test_scores_output(dut):
    """Verify scores_flat bus carries non-zero values after computation."""
    await setup(dut)
    features = [5] * NUM_CELLS
    weights = [[10, 0xF6, 0, 3] for _ in range(NUM_CELLS)]
    await run_inference(dut, features, weights)
    scores_raw = int(dut.scores_flat.value)
    assert scores_raw != 0, "scores_flat should be non-zero"

import math

from amnis_controller import axes_to_command_values, normalize_axes


def almost_equal(seq_a, seq_b, tol=1e-6):
    if len(seq_a) != len(seq_b):
        return False
    return all(math.isclose(a, b, abs_tol=tol) for a, b in zip(seq_a, seq_b))


def test_clamps_to_unit_interval():
    normalized = normalize_axes([-1.5, -0.5, 0.25, 1.5], trigger_axes=[], deadzone=0.0)
    assert almost_equal(normalized, [-1.0, -0.5, 0.25, 1.0])


def test_triggers_mapped_from_zero_to_one():
    normalized = normalize_axes([0.0, 0.5, 1.0], trigger_axes=[1], deadzone=0.0)
    assert almost_equal(normalized, [0.0, 0.0, 1.0])


def test_deadzone_zeroes_small_values():
    normalized = normalize_axes([0.01, -0.03, 0.2], trigger_axes=[], deadzone=0.05)
    assert almost_equal(normalized, [0.0, 0.0, 0.2])


def test_triggers_with_deadzone():
    normalized = normalize_axes([0.48, 0.5, 0.6], trigger_axes=[0, 1, 2], deadzone=0.05)
    # Values near the midpoint should fall into the deadzone after normalization
    assert almost_equal(normalized, [0.0, 0.0, 0.2], tol=1e-3)


def test_axes_to_command_values_defaults():
    # Simulate neutral sticks, RT fully pressed, LT released, gear centered
    normalized = [0.0] * 8
    normalized[5] = 1.0  # Right trigger fully pressed after normalization
    throttle, steer, gear, brake, cmd = axes_to_command_values(normalized)

    assert math.isclose(throttle, 1.0)
    assert math.isclose(steer, 0.0)
    assert gear == 0
    assert math.isclose(brake, 0.0)
    assert cmd == 0


def test_axes_to_command_values_brake_and_gear():
    normalized = [0.0] * 8
    normalized[2] = 1.0  # Left trigger fully pressed after normalization
    normalized[0] = -0.75  # Steering left
    normalized[7] = -1.0  # D-pad down (reverse)

    throttle, steer, gear, brake, cmd = axes_to_command_values(normalized)

    assert math.isclose(throttle, 0.0)
    assert math.isclose(steer, -0.75)
    assert gear == -1
    assert math.isclose(brake, 1.0)
    assert cmd == 0

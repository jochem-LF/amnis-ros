"""Utility helpers for normalizing joystick inputs.

Xbox controller axes report values in different ranges:
- Analog sticks (axes 0, 1, 3, 4): range [-1.0, 1.0]
- Triggers (axes 2, 5): range [0.0, 1.0]
- D-pad (axes 6, 7): range [-1.0, 1.0]

This module converts all axes to a consistent [-1.0, 1.0] range,
which makes it easier for downstream code (like vehicle controllers)
to handle different input types uniformly.
"""
from __future__ import annotations

from typing import Iterable, List, Sequence, Set, Tuple


def _clamp(value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
    """Ensure a value stays within [minimum, maximum].
    
    This is a safety function that keeps values in the valid range.
    If someone passes a value outside the expected range, this keeps it valid.
    
    Args:
        value: The value to constrain
        minimum: The lowest allowed value (default -1.0)
        maximum: The highest allowed value (default 1.0)
    
    Returns:
        The value, clamped to [minimum, maximum]
    """
    return max(min(value, maximum), minimum)


def normalize_axes(
    raw_axes: Sequence[float],
    trigger_axes: Iterable[int] | None = None,
    deadzone: float = 0.0,
) -> List[float]:
    """Normalize joystick axes to the range [-1.0, 1.0].

    This function handles the special case where trigger axes report [0.0, 1.0]
    and converts them to [-1.0, 1.0] to match other axes. It also applies
    a deadzone to filter out small values that might be noise/drift.
    
    Example:
        Xbox controller with sticks and triggers:
        - Stick left: (-0.95, 0.0)
        - Stick right: (0.95, 0.0)
        - Left trigger (axis 2): 0.5 input → -0.0 output after normalization
        - Right trigger (axis 5): 1.0 input → 1.0 output after normalization

    Args:
        raw_axes: The raw axis values from Joy.axes
                  (typically a list of floats from the joystick driver)
        
        trigger_axes: Optional iterable of axis INDICES that represent triggers.
                      For Xbox: typically [2, 5]
                      These axes report [0, 1] and need special handling.
                      Default is None (no trigger axes)
        
        deadzone: Values whose absolute magnitude is below this threshold
                  are forced to 0.0 to avoid drift. Typical value: 0.05.
                  Default is 0.0 (no deadzone filtering)

    Returns:
        List of normalized axis values, each in the range [-1.0, 1.0],
        with small values (< deadzone) replaced with 0.0
    """

    # Convert trigger_axes to a set for fast lookup
    # Only include indices >= 0 (negative indices don't make sense)
    triggers: Set[int] = set(a for a in trigger_axes or [] if a >= 0)
    
    # Ensure deadzone isn't negative (negative deadzone doesn't make sense)
    deadzone = max(deadzone, 0.0)

    # Process each axis value
    normalized: List[float] = []
    for idx, value in enumerate(raw_axes):
        # Check if this axis is a trigger (reports [0, 1] instead of [-1, 1])
        if idx in triggers and 0.0 <= value <= 1.0:
            # Convert from [0, 1] to [-1, 1]
            # This maps: 0.0 → -1.0, 0.5 → 0.0, 1.0 → 1.0
            processed = value * 2.0 - 1.0
        else:
            # For regular axes, just clamp to [-1, 1]
            # (in case of unexpected values from the driver)
            processed = _clamp(value)

        # Apply deadzone: if the value is very small, treat it as 0
        # This eliminates controller drift where sticks slowly move on their own
        if abs(processed) < deadzone:
            processed = 0.0

        # Final safety clamp, then store the result
        normalized.append(_clamp(processed))

    return normalized


def axes_to_command_values(
    normalized_axes: Sequence[float],
    buttons: Sequence[int] = (),
    *,
    throttle_axis: int = 5,
    steer_axis: int = 0,
    brake_axis: int = 4,
    gear_button: int = 9,
    gear_1_button: int = 11,
    gear_minus_1_button: int = 12,
    gear_0_button: int = 13,
    current_gear: int = 0,
) -> Tuple[float, float, int, float, int]:
    """Convert normalized joystick axes into vehicle command semantics.

    The default mapping assumes a 6-axis, 21-button controller layout:

    - ``steer_axis``: steering axis (index 0) - inverted so right=+1, left=-1
    - ``throttle_axis``: throttle axis (index 5) - range 0 (not pressed) to -1 (fully pressed)
    - ``brake_axis``: brake axis (index 4) - range 0 (not pressed) to -1 (fully pressed)
    - ``gear_button``: gear enable button (index 9) - must be pressed to change gears
    - ``gear_1_button``: button to set gear to 1 (index 11)
    - ``gear_minus_1_button``: button to set gear to -1 (index 12)
    - ``gear_0_button``: button to set gear to 0 (index 13)

    Args:
        normalized_axes: Sequence of axes already normalized to [-1.0, 1.0].
        buttons: Sequence of button states (0=not pressed, 1=pressed).
        throttle_axis: Axis index providing throttle (default 5).
        steer_axis: Axis index providing steering (default 0).
        brake_axis: Axis index providing braking (default 4).
        gear_button: Button index that must be pressed to change gears (default 9).
        gear_1_button: Button index to set gear to 1 (default 11).
        gear_minus_1_button: Button index to set gear to -1 (default 12).
        gear_0_button: Button index to set gear to 0 (default 13).
        current_gear: Current gear state for toggle behavior (default 0).

    Returns:
        Tuple containing ``(throttle, steer, gear, brake, cmd)`` where:

        - ``throttle`` is clamped to [0.0, 1.0] (0=not pressed, 1=fully pressed)
        - ``steer`` is clamped to [-1.0, 1.0] (right=+1, left=-1)
        - ``gear`` is an integer in {-1, 0, 1} (requires gear_button press)
        - ``brake`` is clamped to [0.0, 1.0] (0=no brake, 1=full brake)
        - ``cmd`` is an integer command placeholder (always 0 for now)
    """

    def _safe_axis(index: int, default: float) -> float:
        if index < 0 or index >= len(normalized_axes):
            return default
        return float(normalized_axes[index])
    
    def _safe_button(index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False
        return bool(buttons[index])

    # Steering: invert so right is +1, left is -1
    steer_raw = _safe_axis(steer_axis, 0.0)
    steer = _clamp(-steer_raw)

    # Throttle: convert from [0, -1] range to [0.0, 1.0]
    # where 0 = no throttle, -1 = full throttle
    throttle_raw = _safe_axis(throttle_axis, 0.0)  # Default to no throttle
    throttle = _clamp(-throttle_raw, 0.0, 1.0)

    # Brake: convert from [0, -1] range to [0.0, 1.0]
    # where 0 = no brake, -1 = full brake
    brake_raw = _safe_axis(brake_axis, 0.0)  # Default to no brake
    brake = _clamp(-brake_raw, 0.0, 1.0)

    # Gear: button-based selection (only when gear_button is pressed)
    # Check if the gear button is pressed
    gear_button_pressed = _safe_button(gear_button)
    
    if gear_button_pressed:
        # Check which gear button is pressed
        if _safe_button(gear_1_button):
            gear = 1
        elif _safe_button(gear_minus_1_button):
            gear = -1
        elif _safe_button(gear_0_button):
            gear = 0
        else:
            # No gear button pressed, keep current gear
            gear = current_gear
    else:
        # Gear button not pressed, keep current gear
        gear = current_gear

    cmd = 0

    return throttle, steer, gear, brake, cmd


__all__ = ['normalize_axes', 'axes_to_command_values']


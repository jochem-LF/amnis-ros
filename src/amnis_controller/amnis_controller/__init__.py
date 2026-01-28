"""amnis_controller package."""

from .normalization import axes_to_command_values, normalize_axes

__all__ = ['normalize_axes', 'axes_to_command_values']

try:
    from .joystick_normalizer_node import JoystickNormalizerNode
except ImportError:
    JoystickNormalizerNode = None  # type: ignore[assignment]
else:
    __all__.append('JoystickNormalizerNode')

try:
    from .steer_controller_node import SteerControllerNode
except ImportError:
    SteerControllerNode = None  # type: ignore[assignment]
else:
    __all__.append('SteerControllerNode')

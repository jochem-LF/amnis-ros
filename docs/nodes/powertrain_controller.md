# Powertrain Controller Node

**Node Name:** `powertrain_controller`

The Powertrain Controller node manages the vehicle's throttle system. It converts high-level throttle commands into PWM (Pulse Width Modulation) signals required by the motor controller, interfacing via the Jetson Orin's GPIO.

## Responsibilities

1.  **PWM Signal Generation**: Generates precise PWM signals on the configured GPIO pin to control motor speed.
2.  **Safety Watchdog**: Monitors incoming commands. If no command is received within the timeout period (default 0.5s), the throttle is cut to 0.0 to prevent runaway.
3.  **Command Validation**: Clamps throttle values to the configured maximum (`max_throttle`) and applies a deadzone to filter noise.
4.  **Diagnostics**: Publishes real-time status including current throttle, gear, and hardware connection health.

## Subscribed Topics

| Topic                | Type                                     | Description                                               |
| :------------------- | :--------------------------------------- | :-------------------------------------------------------- |
| `powertrain_command` | `amnis_controller/msg/PowertrainCommand` | Input command containing `throttle` (0.0-1.0) and `gear`. |

## Published Topics

| Topic                    | Type                  | Description                                                                    |
| :----------------------- | :-------------------- | :----------------------------------------------------------------------------- |
| `powertrain_diagnostics` | `std_msgs/msg/String` | Diagnostic info including throttle level, connection status, and error counts. |

## Parameters

| Parameter             | Type     | Default                | Description                                     |
| :-------------------- | :------- | :--------------------- | :---------------------------------------------- |
| `input_topic`         | `string` | `'powertrain_command'` | Input topic name.                               |
| `pwm_pin`             | `int`    | `15`                   | Physical GPIO pin number for PWM output.        |
| `pwm_frequency`       | `int`    | `1000`                 | PWM frequency in Hz.                            |
| `max_throttle`        | `double` | `1.0`                  | Maximum allowed throttle (0.0 to 1.0).          |
| `command_timeout_sec` | `double` | `0.5`                  | Safety timeout; cuts throttle if exceeded.      |
| `deadzone`            | `double` | `0.01`                 | Minimum command value to activate motor.        |
| `mock_mode`           | `bool`   | `False`                | Set to `True` to simulate hardware for testing. |

# Joystick Command

**Topic Name:** `vehicle_controller_command`
**Message Type:** `amnis_controller/msg/JoystickCommand`

This topic carries the normalized and aggregated driver inputs from the joystick (or other input devices) to the main vehicle controller. It serves as the primary control interface for the vehicle in "External" mode.

## Message Definition

```text
float32 throttle
float32 steer
int8 gear
float32 brake
uint8 cmd
```

## Fields

| Field      | Type      | Range         | Description                                                            |
| :--------- | :-------- | :------------ | :--------------------------------------------------------------------- |
| `throttle` | `float32` | `[0.0, 1.0]`  | Normalized throttle command. `0.0` is idle, `1.0` is full throttle.    |
| `steer`    | `float32` | `[-1.0, 1.0]` | Normalized steering command. `-1.0` is full left, `1.0` is full right. |
| `gear`     | `int8`    | `{-1, 0, 1}`  | Gear selection. `-1`: Reverse, `0`: Neutral, `1`: Drive.               |
| `brake`    | `float32` | `[0.0, 1.0]`  | Normalized brake command. `0.0` is released, `1.0` is full brake.      |
| `cmd`      | `uint8`   | `0-255`       | Auxiliary command flags (reserved for future use, e.g., horn, lights). |

## Publishers

- [Joystick Normalizer](../nodes/joystick_normalizer.md)

## Subscribers

- [Vehicle Controller](../nodes/vehicle_controller.md)

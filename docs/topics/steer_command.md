# Steer Command

**Topic Name:** `steer_command`
**Message Type:** `amnis_controller/msg/SteerCommand`

This topic carries the steering angle command from the central vehicle controller to the steering subsystem.

## Message Definition

```text
float32 steer
```

## Fields

| Field   | Type      | Range         | Description                                                            |
| :------ | :-------- | :------------ | :--------------------------------------------------------------------- |
| `steer` | `float32` | `[-1.0, 1.0]` | Normalized steering command. `-1.0` is full left, `1.0` is full right. |

## Publishers

- [Vehicle Controller](../nodes/vehicle_controller.md)

## Subscribers

- [Steer Controller](../nodes/steer_controller.md)

# Powertrain Command

**Topic Name:** `powertrain_command`
**Message Type:** `amnis_controller/msg/PowertrainCommand`

This topic carries the throttle and gear commands from the central vehicle controller to the powertrain subsystem.

## Message Definition

```text
float32 throttle
int8 gear
```

## Fields

| Field      | Type      | Range        | Description                                                         |
| :--------- | :-------- | :----------- | :------------------------------------------------------------------ |
| `throttle` | `float32` | `[0.0, 1.0]` | Normalized throttle command. `0.0` is idle, `1.0` is full throttle. |
| `gear`     | `int8`    | `{-1, 0, 1}` | Gear selection. `-1`: Reverse, `0`: Neutral, `1`: Drive.            |

## Publishers

- [Vehicle Controller](../nodes/vehicle_controller.md)

## Subscribers

- [Powertrain Controller](../nodes/powertrain_controller.md)
- [Topic Aggregator](../nodes/topic_aggregator.md) (for dashboard visualization)

# Brake Command

**Topic Name:** `brake_command`
**Message Type:** `amnis_controller/msg/BrakeCommand`

This topic carries the normalized braking command for the vehicle's Electro-Hydraulic Brake (EHB) system.

## Message Definition

```text
float32 brake
```

## Fields

| Field   | Type      | Range        | Description                                                                 |
| :------ | :-------- | :----------- | :-------------------------------------------------------------------------- |
| `brake` | `float32` | `[0.0, 1.0]` | Normalized brake pressure. `0.0` is fully released, `1.0` is fully applied. |

## Publishers

- [Vehicle Controller](../nodes/vehicle_controller.md)

## Subscribers

- [Brake Controller](../nodes/brake_controller.md)

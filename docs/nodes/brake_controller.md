# Brake Controller Node

**Node Name:** `brake_controller`

The Brake Controller node manages the vehicle's Electro-Hydraulic Brake (EHB) system. It translates high-level braking commands into specific CAN bus messages required by the EHB hardware.

## Responsibilities

1.  **Command Translation**: Converts normalized brake commands (`0.0` to `1.0`) into the specific pressure values required by the EHB system.
2.  **CAN Communication**: Interfaces with the hardware via the `socketcan` interface using the `EHBDriver`.
3.  **Safety Watchdog**: Monitors incoming commands. If no command is received within a specified timeout (default 0.2s), it automatically releases the brakes to prevent locking up or undefined behavior.
4.  **Diagnostics**: Publishes real-time status of the brake pressure and CAN connection health.

## Subscribed Topics

| Topic           | Type                                | Description                                     |
| :-------------- | :---------------------------------- | :---------------------------------------------- |
| `brake_command` | `amnis_controller/msg/BrakeCommand` | Normalized brake command (`brake`: 0.0 to 1.0). |

## Published Topics

| Topic               | Type                  | Description                                                   |
| :------------------ | :-------------------- | :------------------------------------------------------------ |
| `brake_diagnostics` | `std_msgs/msg/String` | Diagnostic info including pressure, CAN status, and timeouts. |

## Parameters

| Parameter             | Type     | Default           | Description                                               |
| :-------------------- | :------- | :---------------- | :-------------------------------------------------------- |
| `input_topic`         | `string` | `'brake_command'` | Input topic name.                                         |
| `can_channel`         | `string` | `'can2'`          | CAN interface name (e.g., `can0`, `can1`).                |
| `can_interface`       | `string` | `'socketcan'`     | Type of CAN interface.                                    |
| `pressure_scale`      | `double` | `40.0`            | Scaling factor to convert normalized command to pressure. |
| `command_timeout_sec` | `double` | `0.5`             | Safety timeout; releases brake if exceeded.               |
| `deadzone`            | `double` | `0.01`            | Minimum command value to actuate brakes.                  |
| `mock_mode`           | `bool`   | `False`           | Set to `True` to simulate hardware for testing.           |

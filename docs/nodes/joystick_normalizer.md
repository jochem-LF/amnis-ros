# Joystick Normalizer Node

**Node Name:** `joystick_normalizer`

The Joystick Normalizer node serves as the bridge between the raw controller output (from `game_controller_node`) and the vehicle's main controller. Its primary responsibility is to subscribe to the `/joy` topic, process the raw data, and publish clean, standardized commands.

## Responsibilities

1.  **Normalization**: Converts all joystick axis values to a uniform range.
    - Analog sticks: `[-1.0, 1.0]`
    - Triggers: Mapped to `[0.0, 1.0]` (from their raw `[0, -1]` or `[0, 1]` ranges).
2.  **Deadzone Filtering**: Applies a deadzone to analog stick axes to filter out minor noise or "drift," preventing unintended movement when the controller is idle.
3.  **Command Mapping**: Maps normalized values to specific vehicle actions (Throttle, Steer, Brake, Gear).

## Subscribed Topics

| Topic  | Type                  | Description                                |
| :----- | :-------------------- | :----------------------------------------- |
| `/joy` | `sensor_msgs/msg/Joy` | Raw input from the game controller driver. |

## Published Topics

| Topic                        | Type                                   | Description                                              |
| :--------------------------- | :------------------------------------- | :------------------------------------------------------- |
| `vehicle_controller_command` | `amnis_controller/msg/JoystickCommand` | Standardized command message for the vehicle controller. |

## Controller Mapping

The node maps Xbox controller inputs to the `JoystickCommand` message fields as follows:

### Steering (`steer`)

- **Input**: Left Joystick (Horizontal Axis 0)
- **Range**: `[-1.0, 1.0]`
- **Description**: Inverted so that moving the stick **Right** produces a positive value (`+1.0`) and **Left** produces a negative value (`-1.0`).

### Throttle (`throttle`)

- **Input**: Right Bumper (Axis 5)
- **Range**: `[0.0, 1.0]`
- **Description**: Controlled by the right bumper. Pushing the bumper down results in full throttle (`1.0`). Released is `0.0`.

### Braking (`brake`)

- **Input**: Left Bumper (Axis 4)
- **Range**: `[0.0, 1.0]`
- **Description**: Controlled by the left bumper. Pushing the bumper down results in full braking (`1.0`). Released is `0.0`.

### Gear Selection (`gear`)

- **Input**: D-Pad + Left Trigger (LT / Button 9)
- **Range**: `{-1, 0, 1}`
- **Description**: Gear selection requires a safety button combination. The **Left Trigger (LT)** (mapped to `gear_button` index 9) must be held down to enable gear changes.
  - **Forward (1)**: Hold LT + D-Pad Up
  - **Neutral (0)**: Hold LT + D-Pad Left
  - **Reverse (-1)**: Hold LT + D-Pad Down

If the enable button is not pressed, the gear state remains unchanged.

## Parameters

| Parameter          | Type     | Default                        | Description                                                |
| :----------------- | :------- | :----------------------------- | :--------------------------------------------------------- |
| `input_topic`      | `string` | `'/joy'`                       | Topic to subscribe to for raw input.                       |
| `output_topic`     | `string` | `'vehicle_controller_command'` | Topic to publish normalized commands to.                   |
| `trigger_axes`     | `int[]`  | `[2, 5]`                       | Indices of axes that act as triggers (reporting `[0, 1]`). |
| `deadzone`         | `double` | `0.05`                         | Threshold below which axis values are treated as 0.        |
| `log_throttle_sec` | `double` | `0.5`                          | Time in seconds between log messages to prevent spam.      |

# Vehicle Controller Node

**Node Name:** `vehicle_controller`

The Vehicle Controller node is the central "brain" of the system. It manages the high-level state of the vehicle, processes input commands, and coordinates the subsystems (powertrain, steering, braking).

## Responsibilities

1.  **State Management**: Maintains the vehicle's operational state (e.g., Manual, External Control, Immobilized).
2.  **Command Routing**: Receives standardized commands from the [Joystick Normalizer](joystick_normalizer.md) and routes them to the appropriate subsystem controllers based on the current state.
3.  **Safety Logic**: Enforces safety constraints, such as zeroing commands when in `MANUAL` mode or handling emergency states.

## State Machine

The controller operates using a finite state machine with the following states:

| State           | Description                                                   | Behavior                                                                              |
| :-------------- | :------------------------------------------------------------ | :------------------------------------------------------------------------------------ |
| **MANUAL**      | Default state. The vehicle is under manual control (or idle). | All electronic commands (throttle, steer, brake) are zeroed out.                      |
| **EXTERNAL**    | External control mode (e.g., Joystick).                       | Commands from the `vehicle_controller_command` topic are forwarded to the subsystems. |
| **IMMOBILIZED** | Vehicle is locked.                                            | All commands are zeroed. _(Implementation pending)_                                   |
| **EHB_ERROR**   | Emergency state for brake failure.                            | All commands are zeroed. _(Implementation pending)_                                   |

### Transitions

Transitions are currently controlled by simulated hardware buttons (parameters):

- **MANUAL --> EXTERNAL**: Occurs when the `mode_button` is **ON** and the `safety_button` is **OFF**.
- **EXTERNAL --> MANUAL**: Occurs when the `mode_button` is turned **OFF**.

## Subscribed Topics

| Topic                        | Type                                   | Description                                                 |
| :--------------------------- | :------------------------------------- | :---------------------------------------------------------- |
| `vehicle_controller_command` | `amnis_controller/msg/JoystickCommand` | Standardized input commands (throttle, steer, brake, gear). |

## Published Topics

| Topic                | Type                                     | Description                             |
| :------------------- | :--------------------------------------- | :-------------------------------------- |
| `powertrain_command` | `amnis_controller/msg/PowertrainCommand` | Commands for the powertrain controller. |
| `steer_command`      | `amnis_controller/msg/SteerCommand`      | Commands for the steering controller.   |
| `brake_command`      | `amnis_controller/msg/BrakeCommand`      | Commands for the brake controller.      |

## Parameters

| Parameter          | Type     | Default                        | Description                                           |
| :----------------- | :------- | :----------------------------- | :---------------------------------------------------- |
| `input_topic`      | `string` | `'vehicle_controller_command'` | Input topic name.                                     |
| `powertrain_topic` | `string` | `'powertrain_command'`         | Output topic for powertrain.                          |
| `steer_topic`      | `string` | `'steer_command'`              | Output topic for steering.                            |
| `brake_topic`      | `string` | `'brake_command'`              | Output topic for braking.                             |
| `safety_button`    | `bool`   | `False`                        | Simulated safety button state (hardware placeholder). |
| `mode_button`      | `bool`   | `False`                        | Simulated mode button state (hardware placeholder).   |

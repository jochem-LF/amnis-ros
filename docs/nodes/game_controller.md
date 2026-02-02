# Game Controller Node

**Node Name:** `game_controller_node` (from package `joy`)

The Game Controller node acts as the driver for the gamepad hardware. It interfaces directly with the Linux input subsystem (e.g., `/dev/input/js0`) to read button presses and axis movements.

## Overview

We utilize the `game_controller_node` from the standard ROS 2 `joy` package. This node captures hardware inputs and translates them into ROS 2 messages.

### Why `game_controller_node`?

Previous iterations of this project used the standard `joy_node`. However, a critical bug was identified where brake and acceleration button values would skyrocket to their maximums immediately upon crossing a certain threshold, causing dangerous sudden acceleration or hard braking.

The `game_controller_node` solves this issue by providing stable, linear values for analog triggers, making it safe for vehicle control.

## Functionality

The node publishes raw, unnormalized data to the `/joy` topic.

- **Topic**: `/joy`
- **Message Type**: `sensor_msgs/msg/Joy`

The message contains:

- `axes`: Array of floating-point numbers (sticks, triggers).
- `buttons`: Array of integers (0 or 1) for button states.

## Data Interpretation

The raw data published by this node is not directly suitable for vehicle control for several reasons:

1.  **Inconsistent Ranges**: Analog sticks typically map to `[-1.0, 1.0]`, while triggers might map to `[0.0, 1.0]` or `[-1.0, 1.0]` depending on the driver.
2.  **Lack of Semantics**: The data is just an array of numbers (e.g., `axes[2]`). It requires mapping to understand that `axes[2]` corresponds to the "Left Trigger".

To address this, the output of this node is sent to the [Joystick Normalizer Node](joystick_normalizer.md), which handles the interpretation and normalization of this raw data into meaningful vehicle commands.

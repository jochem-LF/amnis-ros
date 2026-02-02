# Nodes Overview

This section documents the ROS 2 nodes in the `amnis_controller` package.

## List of Nodes

| Node Name                                         | Description                                                          |
| :------------------------------------------------ | :------------------------------------------------------------------- |
| [Game Controller](game_controller.md)             | Driver for the gamepad hardware (standard `joy` package).            |
| [Joystick Normalizer](joystick_normalizer.md)     | Normalizes the game controller raw inputs to usable outputs          |
| [Vehicle Controller](vehicle_controller.md)       | High-level vehicle state machine and coordination.                   |
| [Steer Controller](steer_controller.md)           | Controls the steering mechanism.                                     |
| [Brake Controller](brake_controller.md)           | Manages the braking system.                                          |
| [Powertrain Controller](powertrain_controller.md) | Controls the vehicle's powertrain (motors).                          |
| [Topic Aggregator](topic_aggregator.md)           | Aggregates and exposes ROS topics over WebSockets for the dashboard. |

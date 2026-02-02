# Launch File Configuration

The primary launch file for the Amnis S7 vehicle is `full_controller.launch.py`. This file orchestrates the startup of all necessary nodes for vehicle control, including hardware drivers, state management, and the user interface.

## Usage

To launch the full system, use the following command:

```bash
ros2 launch amnis_controller full_controller.launch.py
```

## Overview of Launched Nodes

The launch file starts the following nodes in order:

1.  **`game_controller_node`** (`joy` package): Reads raw input from the connected joystick/gamepad.
2.  **`joystick_normalizer_node`**: Normalizes raw joystick data into a standard format.
3.  **`vehicle_controller_node`**: The central brain that manages the vehicle's state machine and safety logic.
4.  **`steer_controller_node`**: Controls the steering motor via an I2C H-bridge driver.
5.  **`brake_controller_node`**: Manages the Electronic Hydraulic Brake (EHB) system via CAN bus.
6.  **`powertrain_controller_node`**: Controls the vehicle's throttle via PWM signals.
7.  **`topic_aggregator_node`**: Bridges ROS 2 topics to a WebSocket server for the frontend dashboard.
8.  **Firefox Dashboard**: Automatically opens the `dashboard.html` interface in kiosk mode.

## Configuration & Parameters

The launch file defines several key parameters for each node. These can be modified directly in the launch file or overridden at runtime.

### Joystick Configuration

- **`device_id`**: `0` (Default for `/dev/input/js0`)
- **`deadzone`**: `0.05`

### Steer Controller

- **`i2c_bus`**: `7` (Jetson Orin I2C bus)
- **`i2c_address`**: `0x58` (H-bridge address)
- **`max_power`**: `100` (Max speed percentage)
- **`mock_mode`**: `False` (Set to `True` to test without hardware)

### Brake Controller

- **`can_channel`**: `'can1'`
- **`can_interface`**: `'socketcan'`
- **`pressure_scale`**: `40.0`
- **`mock_mode`**: `False`

### Powertrain Controller

- **`pwm_pin`**: `15` (Physical GPIO pin)
- **`pwm_frequency`**: `1000` Hz
- **`max_throttle`**: `1.0` (Safety limit, 0.0-1.0)
- **`mock_mode`**: `False`

### Topic Aggregator

- **`websocket_port`**: `8765`
- **`update_frequency_hz`**: `30.0`

## Dashboard

The launch file automatically attempts to open `dashboard.html` from the workspace root using Firefox in kiosk mode. Ensure Firefox is installed.

# Web Dashboard

The Amnis S7 Dashboard is a standalone HTML/JS application that provides real-time visualization and monitoring of the vehicle's state. It runs in a web browser (Firefox Kiosk mode) and communicates with the ROS 2 system via the [Topic Aggregator](../nodes/topic_aggregator.md).

## Features

- **Real-time Visualization**:
  - **Steering**: Visual steering wheel indicator.
  - **Throttle**: Gauge showing current throttle application.
  - **Braking**: Bar indicator for brake pressure.
- **System Status**:
  - **Connection Health**: Indicators for WebSocket and ROS topic latency.
  - **Vehicle State**: Displays current mode (Manual, External, etc.).
- **Diagnostics**:
  - Detailed views for Powertrain, Steering, and Brake subsystems.
  - Raw data inspector for debugging message contents.

## Architecture

The dashboard is a "Single File Application" (`dashboard.html`) located in the root of the workspace. It uses vanilla JavaScript and CSS, requiring no build step.

1.  **Launch**: The `full_controller.launch.py` file launches Firefox in kiosk mode pointing to this file.
2.  **Connection**: On load, it attempts to connect to `ws://localhost:8765` (the Topic Aggregator).
3.  **Data Flow**:
    - Receives JSON updates from the aggregator.
    - Parses the data and updates the DOM elements (gauges, text, classes).

## Usage

The dashboard is automatically started by the main launch file. However, you can also open it manually in any browser on the vehicle computer (or a computer on the same network) by opening `dashboard.html`.

If running remotely, you may need to update the WebSocket URL in the dashboard code or ensure port `8765` is forwarded.

## Views

The dashboard has three main views, toggleable via the navigation button:

1.  **Main View**: High-level driver info (Steering, Throttle, Brake).
2.  **Details View**: In-depth diagnostics and state information.
3.  **Raw Data View**: A scrolling list of the raw JSON messages received.

## Customization

The dashboard styling is defined in the `<style>` section of `dashboard.html`. It uses a dark, high-contrast theme suitable for in-vehicle displays.

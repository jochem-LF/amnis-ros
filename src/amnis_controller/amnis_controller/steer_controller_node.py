#!/usr/bin/env python3
"""Steer controller node for controlling H-bridge motor driver via I2C.

This node subscribes to SteerCommand messages and translates them into
I2C commands for an H-bridge motor driver. It includes safety features,
error handling, and diagnostics.

Author: amnis_controller
Target: Jetson Orin (or any Linux system with I2C)
"""

from typing import Sequence
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from amnis_controller.msg import SteerCommand

from amnis_controller.drivers import HBridgeDriver


class SteerControllerNode(Node):
    """ROS2 node for steering control via H-bridge motor driver.
    
    This node:
    - Subscribes to /steer_command topic
    - Validates and limits steering commands
    - Controls H-bridge via I2C
    - Publishes diagnostics
    - Implements safety watchdog (stops if no command received)
    """

    def __init__(self) -> None:
        """Initialize the steer controller node."""
        super().__init__('steer_controller')

        # Declare parameters
        self.declare_parameter('input_topic', 'steer_command')
        self.declare_parameter('diagnostic_topic', 'steer_diagnostics')
        
        # Hardware configuration
        self.declare_parameter('i2c_bus', 1)  # Raspberry Pi typically uses bus 1
        self.declare_parameter('i2c_address', 0x58)  # H-bridge I2C address
        self.declare_parameter('max_power', 100)
        self.declare_parameter('mock_mode', False)  # For testing without hardware
        
        # Pigpio connection configuration (for remote Raspberry Pi)
        self.declare_parameter('pigpio_host', 'localhost')
        self.declare_parameter('pigpio_port', 8888)
        
        # Safety parameters
        self.declare_parameter('command_timeout_sec', 0.5)  # Stop if no command
        self.declare_parameter('deadzone', 0.05)  # Ignore small commands
        self.declare_parameter('auto_stop_on_error', True)  # Auto-stop on I2C errors
        self.declare_parameter('error_threshold', 3)  # Errors before auto-stop
        
        # Control parameters
        self.declare_parameter('update_rate_hz', 50.0)  # I2C update rate
        self.declare_parameter('steer_to_power_scale', 100.0)  # Maps steer [-1,1] to power
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)  # Enable/disable info logging
        
        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        max_power = self.get_parameter('max_power').value
        mock_mode = self.get_parameter('mock_mode').value
        pigpio_host = self.get_parameter('pigpio_host').value
        pigpio_port = self.get_parameter('pigpio_port').value
        self.command_timeout = self.get_parameter('command_timeout_sec').value
        self.deadzone = self.get_parameter('deadzone').value
        auto_stop_on_error = self.get_parameter('auto_stop_on_error').value
        error_threshold = self.get_parameter('error_threshold').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.steer_scale = self.get_parameter('steer_to_power_scale').value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        # Initialize hardware driver
        self.driver = HBridgeDriver(
            i2c_bus=i2c_bus,
            i2c_address=i2c_address,
            max_power=max_power,
            mock_mode=mock_mode,
            pigpio_host=pigpio_host,
            pigpio_port=pigpio_port,
            auto_stop_on_error=auto_stop_on_error,
            error_threshold=error_threshold,
        )
        
        if not self.driver.is_connected():
            self.get_logger().error(
                "Failed to initialize H-bridge driver! Running in degraded mode."
            )
        
        # State tracking
        self._last_command: SteerCommand | None = None
        self._last_command_time: Time | None = None
        self._current_direction = 0
        self._current_speed = 0
        self._is_timed_out = False
        self._last_error_count = 0
        
        # Create subscriber
        self.subscription = self.create_subscription(
            SteerCommand,
            input_topic,
            self.steer_command_callback,
            10
        )
        
        # Create diagnostics publisher
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(
                String,
                diagnostic_topic,
                10
            )
        
        # Create update timer for periodic I2C updates
        update_period = 1.0 / update_rate
        self.update_timer = self.create_timer(
            update_period,
            self.update_callback
        )
        
        # Create watchdog timer for safety timeout
        self.watchdog_timer = self.create_timer(
            0.1,  # Check every 100ms
            self.watchdog_callback
        )
        
        # Logging timer
        self.last_log_time = self.get_clock().now()
        
        if self.verbose:
            self.get_logger().info(
                f"Steer controller initialized: "
                f"topic={input_topic}, "
                f"i2c_bus={i2c_bus}, "
                f"i2c_addr=0x{i2c_address:02x}, "
                f"mock={mock_mode}"
            )
    
    def steer_command_callback(self, msg: SteerCommand) -> None:
        """Handle incoming steering commands.
        
        Args:
            msg: SteerCommand message with steer value [-1, 1]
        """
        self._last_command = msg
        self._last_command_time = self.get_clock().now()
        self._is_timed_out = False
    
    def update_callback(self) -> None:
        """Periodic update to send commands to hardware.
        
        This runs at a fixed rate (e.g., 50Hz) to maintain smooth control.
        """
        # Check if we have a command
        if self._last_command is None:
            return
        
        # Apply deadzone
        steer = self._last_command.steer
        if abs(steer) < self.deadzone:
            steer = 0.0
        
        # Convert steer [-1, 1] to direction and speed
        # For steering: positive = right (2), negative = left (1)
        if steer > 0:
            direction = 2  # Right
            speed = int(steer * self.steer_scale)
        elif steer < 0:
            direction = 1  # Left
            speed = int(-steer * self.steer_scale)
        else:
            direction = 0  # Stop
            speed = 0
        
        # Send to hardware - simple, no blocking
        success = self.driver.set_direction_speed(direction, speed)
        
        if success:
            self._current_direction = direction
            self._current_speed = speed
        
        # Detect new errors (just for logging)
        current_errors = self.driver.get_error_count()
        if current_errors > self._last_error_count:
            new_errors = current_errors - self._last_error_count
            consecutive = self.driver.get_consecutive_errors()
            if consecutive >= 3:
                self.get_logger().warning(
                    f"I2C errors: {consecutive} consecutive (likely hit steering limit)"
                )
        self._last_error_count = current_errors
        
        # Periodic logging
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                dir_name = {0: "STOP", 1: "RIGHT", 2: "LEFT"}.get(direction, "?")
                self.get_logger().info(
                    f"Steer: {steer:.3f} → Dir: {direction}({dir_name}), Speed: {speed}% | "
                    f"Connected: {self.driver.is_connected()} | "
                    f"Errors: {self.driver.get_error_count()}"
                )
                self.last_log_time = now
        
        # Publish diagnostics
        if self.publish_diagnostics:
            self._publish_diagnostics(steer, direction, speed, success)
    
    def watchdog_callback(self) -> None:
        """Safety watchdog - stops motor if no command received recently."""
        if self._last_command_time is None:
            return
        
        now = self.get_clock().now()
        time_since_command = (now - self._last_command_time).nanoseconds / 1e9
        
        if time_since_command > self.command_timeout and not self._is_timed_out:
            self.get_logger().warning(
                f"Command timeout ({time_since_command:.2f}s) - stopping motor!"
            )
            self.driver.stop()
            self._current_direction = 0
            self._current_speed = 0
            self._is_timed_out = True
    
    def _publish_diagnostics(
        self,
        steer: float,
        direction: int,
        speed: int,
        success: bool
    ) -> None:
        """Publish diagnostic information.
        
        Args:
            steer: Commanded steer value
            direction: Converted direction (0/1/2)
            speed: Converted speed (0-100)
            success: Whether command was successful
        """
        msg = String()
        dir_name = {0: "STOP", 1: "RIGHT", 2: "LEFT"}.get(direction, "UNKNOWN")
        msg.data = (
            f"steer={steer:.3f}, "
            f"direction={direction}({dir_name}), "
            f"speed={speed}, "
            f"connected={self.driver.is_connected()}, "
            f"success={success}, "
            f"errors={self.driver.get_error_count()}, "
            f"consecutive_errors={self.driver.get_consecutive_errors()}, "
            f"timed_out={self._is_timed_out}"
        )
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed.
        
        Ensures motor is stopped and sends final stop command to H-bridge.
        """
        if self.verbose:
            self.get_logger().info("Shutting down steer controller...")
        
        # Stop motor before shutdown (sends direction=0, speed=0)
        self.driver.stop()
        
        # Close hardware connection (also sends final stop command)
        self.driver.close()
        
        # Call parent cleanup
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node when run as a standalone executable.
    
    This function is called when you run:
        ros2 run amnis_controller steer_controller_node
    
    It:
    1. Initializes ROS 2 (rclpy.init)
    2. Creates an instance of our node
    3. Spins the node (runs the main loop that processes callbacks)
    4. Cleans up when interrupted
    
    Args:
        args: Command line arguments (passed to rclpy.init)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create node instance
    node = SteerControllerNode()
    
    try:
        # Spin keeps the node running and processing callbacks
        # It will block here until Ctrl+C or rclpy.shutdown()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C - always log this
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        # Always clean up
        node.destroy_node()
        
        # Only shutdown if rclpy is still initialized
        # (prevents error if shutdown was called elsewhere)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()


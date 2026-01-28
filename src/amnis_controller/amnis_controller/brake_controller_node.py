#!/usr/bin/env python3
"""Brake controller node for controlling EHB (Electro-Hydraulic Brake) via CAN bus.

This node subscribes to BrakeCommand messages and translates them into
CAN commands for the electro-hydraulic brake system. It includes safety features,
error handling, and diagnostics.

Author: amnis_controller
Target: Systems with CAN bus support (socketcan on Linux)
"""

from typing import Sequence
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from amnis_controller.msg import BrakeCommand

from amnis_controller.drivers import EHBDriver


class BrakeControllerNode(Node):
    """ROS2 node for brake control via EHB system over CAN bus.
    
    This node:
    - Subscribes to /brake_command topic
    - Validates and limits brake commands
    - Controls EHB system via CAN bus
    - Publishes diagnostics
    - Implements safety watchdog (releases brake if no command received)
    """

    def __init__(self) -> None:
        """Initialize the brake controller node."""
        super().__init__('brake_controller')

        # Declare parameters
        self.declare_parameter('input_topic', 'brake_command')
        self.declare_parameter('diagnostic_topic', 'brake_diagnostics')
        
        # Hardware configuration
        self.declare_parameter('can_channel', 'can2')
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('pressure_scale', 40.0)
        self.declare_parameter('mock_mode', False)  # For testing without hardware
        
        # Safety parameters
        self.declare_parameter('command_timeout_sec', 0.5)  # Release brake if no command
        self.declare_parameter('deadzone', 0.01)  # Ignore small brake commands
        
        # Control parameters
        self.declare_parameter('update_rate_hz', 10.0)  # Status update rate
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)  # Enable/disable info logging
        
        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        can_channel = self.get_parameter('can_channel').value
        can_interface = self.get_parameter('can_interface').value
        pressure_scale = self.get_parameter('pressure_scale').value
        mock_mode = self.get_parameter('mock_mode').value
        self.command_timeout = self.get_parameter('command_timeout_sec').value
        self.deadzone = self.get_parameter('deadzone').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        # Initialize hardware driver
        self.driver = EHBDriver(
            can_channel=can_channel,
            can_interface=can_interface,
            pressure_scale=pressure_scale,
            mock_mode=mock_mode
        )
        
        if not self.driver.is_connected():
            self.get_logger().error(
                "Failed to initialize EHB driver! Running in degraded mode."
            )
        
        # State tracking
        self._last_command: BrakeCommand | None = None
        self._last_command_time: Time | None = None
        self._current_pressure = 0.0
        self._is_timed_out = False
        
        # Create subscriber
        self.subscription = self.create_subscription(
            BrakeCommand,
            input_topic,
            self.brake_command_callback,
            10
        )
        
        # Create diagnostics publisher
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(
                String,
                diagnostic_topic,
                10
            )
        
        # Create update timer for periodic status updates and diagnostics
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
                f"Brake controller initialized: "
                f"topic={input_topic}, "
                f"can_channel={can_channel}, "
                f"can_interface={can_interface}, "
                f"mock={mock_mode}"
            )
    
    def brake_command_callback(self, msg: BrakeCommand) -> None:
        """Handle incoming brake commands.
        
        Args:
            msg: BrakeCommand message with brake value [0, 1]
        """
        self._last_command = msg
        self._last_command_time = self.get_clock().now()
        self._is_timed_out = False
        
        # Process brake command immediately
        brake = msg.brake
        
        # Apply deadzone
        if abs(brake) < self.deadzone:
            brake = 0.0
        
        # Clamp to valid range [0.0, 1.0]
        brake = max(0.0, min(1.0, brake))
        
        # Send to hardware
        success = self.driver.set_pressure(brake)
        
        if success:
            self._current_pressure = brake
    
    def update_callback(self) -> None:
        """Periodic update for status monitoring and diagnostics.
        
        This runs at a lower rate than the CAN transmission (which runs
        at 50Hz internally in the driver).
        """
        # Check if we have a command
        if self._last_command is None:
            return
        
        pressure = self._current_pressure
        
        # Periodic logging
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                time_since_msg = self.driver.get_time_since_last_message()
                time_str = f"{time_since_msg:.3f}s" if time_since_msg is not None else "None"
                self.get_logger().info(
                    f"Brake: pressure={pressure:.3f} | "
                    f"CAN OK: {self.driver.has_can_communication()} | "
                    f"Last msg: {time_str} ago"
                )
                self.last_log_time = now
        
        # Publish diagnostics
        if self.publish_diagnostics:
            self._publish_diagnostics(pressure)
    
    def watchdog_callback(self) -> None:
        """Safety watchdog - releases brake if no command received recently."""
        if self._last_command_time is None:
            return
        
        now = self.get_clock().now()
        time_since_command = (now - self._last_command_time).nanoseconds / 1e9
        
        # Check command timeout
        if time_since_command > self.command_timeout and not self._is_timed_out:
            self.get_logger().warning(
                f"Command timeout ({time_since_command:.2f}s) - releasing brake!"
            )
            self.driver.stop()
            self._current_pressure = 0.0
            self._is_timed_out = True
        
        # Check CAN communication - simple: did we receive ANY message in last 0.5s?
        if not self.driver.has_can_communication():
            time_since_msg = self.driver.get_time_since_last_message()
            if time_since_msg is not None:
                self.get_logger().error(
                    f"CAN communication lost! No messages for {time_since_msg:.2f}s"
                )
            else:
                self.get_logger().error(
                    "CAN communication lost! No messages received since startup"
                )
            # Emergency stop
            self.driver.stop()
            self._current_pressure = 0.0
    
    def _publish_diagnostics(self, pressure: float) -> None:
        """Publish diagnostic information.
        
        Args:
            pressure: Current brake pressure [0.0, 1.0]
        """
        msg = String()
        time_since_msg = self.driver.get_time_since_last_message()
        
        # Format time_since_msg properly
        if time_since_msg is not None:
            time_str = f"{time_since_msg:.3f}"
        else:
            time_str = "None"
        
        msg.data = (
            f"brake_pressure={pressure:.3f}, "
            f"can_ok={self.driver.has_can_communication()}, "
            f"time_since_last_msg={time_str}, "
            f"timed_out={self._is_timed_out}"
        )
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed.
        
        Ensures brake is released and CAN communication is properly stopped.
        """
        if self.verbose:
            self.get_logger().info("Shutting down brake controller...")
        
        # Release brake before shutdown
        self.driver.stop()
        
        # Close hardware connection
        self.driver.close()
        
        # Call parent cleanup
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node when run as a standalone executable.
    
    This function is called when you run:
        ros2 run amnis_controller brake_controller_node
    
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
    node = BrakeControllerNode()
    
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


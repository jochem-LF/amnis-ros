#!/usr/bin/env python3
"""Powertrain controller node for controlling throttle via PWM on GPIO.

This node subscribes to PowertrainCommand messages and translates them into
PWM signals for throttle control. It includes safety features, error handling,
and diagnostics.

Author: amnis_controller
Target: Systems with GPIO support (Jetson Orin)
"""

from typing import Sequence
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from amnis_controller.msg import PowertrainCommand

from amnis_controller.drivers import PWMDriver


class PowertrainControllerNode(Node):
    """ROS2 node for powertrain control via PWM on GPIO.
    
    This node:
    - Subscribes to /powertrain_command topic
    - Validates and limits throttle commands
    - Controls throttle via PWM on GPIO pin
    - Publishes diagnostics
    - Implements safety watchdog (cuts throttle if no command received)
    """

    def __init__(self) -> None:
        """Initialize the powertrain controller node."""
        super().__init__('powertrain_controller')

        # Declare parameters
        self.declare_parameter('input_topic', 'powertrain_command')
        self.declare_parameter('diagnostic_topic', 'powertrain_diagnostics')
        
        # Hardware configuration
        self.declare_parameter('pwm_pin', 15)  # Physical pin number
        self.declare_parameter('pwm_frequency', 1000)  # 1kHz
        self.declare_parameter('max_throttle', 1.0)  # Maximum throttle (0.0-1.0)
        self.declare_parameter('mock_mode', False)  # For testing without hardware
        
        # Safety parameters
        self.declare_parameter('command_timeout_sec', 0.5)  # Cut throttle if no command
        self.declare_parameter('deadzone', 0.01)  # Ignore small throttle commands
        
        # Control parameters
        self.declare_parameter('update_rate_hz', 20.0)  # Status update rate
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)  # Enable/disable info logging
        
        # Get parameter values
        input_topic = self.get_parameter('input_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        pwm_pin = self.get_parameter('pwm_pin').value
        pwm_frequency = self.get_parameter('pwm_frequency').value
        max_throttle = self.get_parameter('max_throttle').value
        mock_mode = self.get_parameter('mock_mode').value
        self.command_timeout = self.get_parameter('command_timeout_sec').value
        self.deadzone = self.get_parameter('deadzone').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        # Initialize hardware driver
        self.driver = PWMDriver(
            pwm_pin=pwm_pin,
            pwm_frequency=pwm_frequency,
            max_throttle=max_throttle,
            mock_mode=mock_mode
        )
        
        if not self.driver.is_connected():
            self.get_logger().error(
                "Failed to initialize PWM driver! Running in degraded mode."
            )
        
        # State tracking
        self._last_command: PowertrainCommand | None = None
        self._last_command_time: Time | None = None
        self._current_throttle = 0.0
        self._current_gear = 0
        self._is_timed_out = False
        
        # Create subscriber
        self.subscription = self.create_subscription(
            PowertrainCommand,
            input_topic,
            self.powertrain_command_callback,
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
                f"Powertrain controller initialized: "
                f"topic={input_topic}, "
                f"pwm_pin={pwm_pin}, "
                f"pwm_frequency={pwm_frequency}Hz, "
                f"mock={mock_mode}"
            )
    
    def powertrain_command_callback(self, msg: PowertrainCommand) -> None:
        """Handle incoming powertrain commands.
        
        Args:
            msg: PowertrainCommand message with throttle [0, 1] and gear
        """
        self._last_command = msg
        self._last_command_time = self.get_clock().now()
        self._is_timed_out = False
        
        # Process throttle command immediately
        throttle = msg.throttle
        
        # Apply deadzone
        if abs(throttle) < self.deadzone:
            throttle = 0.0
        
        # Clamp to valid range [0.0, 1.0]
        throttle = max(0.0, min(1.0, throttle))
        
        # Send to hardware
        success = self.driver.set_throttle(throttle)
        
        if success:
            self._current_throttle = throttle
        
        # Store gear (not used yet, but track it)
        self._current_gear = msg.gear
    
    def update_callback(self) -> None:
        """Periodic update for status monitoring and diagnostics.
        
        This runs at a lower rate than the PWM control.
        """
        # Check if we have a command
        if self._last_command is None:
            return
        
        throttle = self._current_throttle
        gear = self._current_gear
        
        # Periodic logging
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                self.get_logger().info(
                    f"Powertrain: throttle={throttle:.3f}, gear={gear} | "
                    f"Connected: {self.driver.is_connected()} | "
                    f"Errors: {self.driver.get_error_count()}"
                )
                self.last_log_time = now
        
        # Publish diagnostics
        if self.publish_diagnostics:
            self._publish_diagnostics(throttle, gear)
    
    def watchdog_callback(self) -> None:
        """Safety watchdog - cuts throttle if no command received recently."""
        if self._last_command_time is None:
            return
        
        now = self.get_clock().now()
        time_since_command = (now - self._last_command_time).nanoseconds / 1e9
        
        if time_since_command > self.command_timeout and not self._is_timed_out:
            self.get_logger().warning(
                f"Command timeout ({time_since_command:.2f}s) - cutting throttle!"
            )
            self.driver.stop()
            self._current_throttle = 0.0
            self._is_timed_out = True
    
    def _publish_diagnostics(self, throttle: float, gear: int) -> None:
        """Publish diagnostic information.
        
        Args:
            throttle: Current throttle [0.0, 1.0]
            gear: Current gear (-1, 0, 1)
        """
        msg = String()
        msg.data = (
            f"throttle={throttle:.3f}, "
            f"gear={gear}, "
            f"connected={self.driver.is_connected()}, "
            f"errors={self.driver.get_error_count()}, "
            f"timed_out={self._is_timed_out}"
        )
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed.
        
        Ensures throttle is cut and GPIO is properly cleaned up.
        """
        if self.verbose:
            self.get_logger().info("Shutting down powertrain controller...")
        
        # Cut throttle before shutdown
        self.driver.stop()
        
        # Close hardware connection
        self.driver.close()
        
        # Call parent cleanup
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node when run as a standalone executable.
    
    This function is called when you run:
        ros2 run amnis_controller powertrain_controller_node
    
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
    node = PowertrainControllerNode()
    
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


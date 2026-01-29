#!/usr/bin/env python3
"""PID steering controller node for closed-loop position control.

This node implements a PID controller that reads the desired steering position
from joystick input and the actual steering position from the ADC sensor, then
outputs motor commands to minimize the error between them.

Author: amnis_controller
"""

from typing import Sequence, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from amnis_controller.msg import JoystickCommand, SteerCommand, SensorData

from amnis_controller.pid_controller import PIDController


class PIDSteerControllerNode(Node):
    """ROS2 node for PID-based closed-loop steering control.
    
    This node:
    - Subscribes to joystick commands (desired steering position)
    - Subscribes to sensor data (actual steering position)
    - Runs a PID controller to minimize error
    - Publishes motor commands to the H-bridge
    """

    def __init__(self) -> None:
        """Initialize the PID steering controller node."""
        super().__init__('pid_steer_controller')

        # Declare parameters
        self.declare_parameter('joystick_topic', 'vehicle_controller_command')
        self.declare_parameter('sensor_topic', 'sensor_data')
        self.declare_parameter('output_topic', 'steer_command')
        self.declare_parameter('diagnostic_topic', 'pid_steer_diagnostics')
        
        # PID parameters
        self.declare_parameter('kp', 2.0)  # Proportional gain
        self.declare_parameter('ki', 0.1)  # Integral gain
        self.declare_parameter('kd', 0.05)  # Derivative gain
        self.declare_parameter('output_limit', 100.0)  # Max motor power %
        self.declare_parameter('integral_limit', 10.0)  # Anti-windup limit
        self.declare_parameter('deadband', 0.02)  # Error threshold (2%)
        
        # Control parameters
        self.declare_parameter('update_rate_hz', 20.0)  # Control loop rate
        self.declare_parameter('sensor_timeout_sec', 0.5)  # Sensor timeout
        
        # Safety parameters
        self.declare_parameter('max_error', 0.5)  # Max acceptable error (50%)
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)
        
        # Get parameter values
        joystick_topic = self.get_parameter('joystick_topic').value
        sensor_topic = self.get_parameter('sensor_topic').value
        output_topic = self.get_parameter('output_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        output_limit = self.get_parameter('output_limit').value
        integral_limit = self.get_parameter('integral_limit').value
        deadband = self.get_parameter('deadband').value
        
        update_rate = self.get_parameter('update_rate_hz').value
        self.sensor_timeout = self.get_parameter('sensor_timeout_sec').value
        self.max_error = self.get_parameter('max_error').value
        
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        # Initialize PID controller
        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            output_limit=output_limit,
            integral_limit=integral_limit,
            deadband=deadband
        )
        
        # State tracking
        self._desired_position: Optional[float] = None
        self._actual_position: Optional[float] = None
        self._last_sensor_time = None
        self._sensor_available = False
        
        # Create subscribers
        self.joystick_sub = self.create_subscription(
            JoystickCommand,
            joystick_topic,
            self.joystick_callback,
            10
        )
        
        self.sensor_sub = self.create_subscription(
            SensorData,
            sensor_topic,
            self.sensor_callback,
            10
        )
        
        # Create publisher
        self.steer_pub = self.create_publisher(
            SteerCommand,
            output_topic,
            10
        )
        
        # Create diagnostics publisher
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(
                String,
                diagnostic_topic,
                10
            )
        
        # Create control loop timer
        update_period = 1.0 / update_rate
        self.control_timer = self.create_timer(
            update_period,
            self.control_loop
        )
        
        # Logging timer
        self.last_log_time = self.get_clock().now()
        
        if self.verbose:
            self.get_logger().info(
                f"PID steering controller initialized: "
                f"Kp={kp}, Ki={ki}, Kd={kd}, "
                f"rate={update_rate}Hz"
            )
    
    def joystick_callback(self, msg: JoystickCommand) -> None:
        """Handle incoming joystick commands (desired position).
        
        Args:
            msg: JoystickCommand message with steer value (0-1 expected after vehicle controller)
        """
        # Store desired position (expecting 0-1 range)
        # The vehicle controller should already normalize this
        self._desired_position = max(0.0, min(1.0, msg.steer))
    
    def sensor_callback(self, msg: SensorData) -> None:
        """Handle incoming sensor data (actual position).
        
        Args:
            msg: SensorData message with steering_wheel value (0-1 normalized)
        """
        # Store actual position from steering wheel sensor
        self._actual_position = msg.steering_wheel
        self._last_sensor_time = self.get_clock().now()
        self._sensor_available = True
    
    def control_loop(self) -> None:
        """Main PID control loop (runs at update_rate_hz)."""
        
        # Check if we have necessary data
        if self._desired_position is None:
            return
        
        # Check sensor timeout
        if self._last_sensor_time is not None:
            sensor_age = (self.get_clock().now() - self._last_sensor_time).nanoseconds / 1e9
            if sensor_age > self.sensor_timeout:
                if self._sensor_available:
                    self.get_logger().warning(
                        f"Sensor timeout ({sensor_age:.2f}s) - using open-loop control"
                    )
                    self._sensor_available = False
        
        # If no sensor data, fall back to open-loop (direct passthrough)
        if not self._sensor_available or self._actual_position is None:
            self._open_loop_control()
            return
        
        # Calculate error (desired - actual)
        error = self._desired_position - self._actual_position
        
        # Safety check: if error is too large, something is wrong
        if abs(error) > self.max_error:
            if self.verbose:
                self.get_logger().warning(
                    f"Large error detected: {error:.3f} (desired={self._desired_position:.3f}, "
                    f"actual={self._actual_position:.3f})"
                )
        
        # Run PID controller
        motor_power = self.pid.update(error)
        
        # Convert power to direction and speed
        direction, speed = self._power_to_direction_speed(motor_power)
        
        # Publish steering command
        steer_msg = SteerCommand()
        steer_msg.steer = motor_power / 100.0  # Normalize for compatibility
        self.steer_pub.publish(steer_msg)
        
        # Logging
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                p, i, d = self.pid.get_terms()
                self.get_logger().info(
                    f"PID: desired={self._desired_position:.3f}, actual={self._actual_position:.3f}, "
                    f"error={error:.3f}, power={motor_power:.1f}% "
                    f"(P={p:.1f}, I={i:.1f}, D={d:.1f})"
                )
                self.last_log_time = now
        
        # Publish diagnostics
        if self.publish_diagnostics:
            self._publish_diagnostics(error, motor_power)
    
    def _open_loop_control(self) -> None:
        """Fall back to open-loop control when sensor unavailable.
        
        Maps desired position (0-1) directly to motor command.
        """
        # Map 0-1 to motor command
        # 0.5 = center (no movement)
        # < 0.5 = turn left
        # > 0.5 = turn right
        center = 0.5
        error_from_center = self._desired_position - center
        
        # Scale to motor power (-100 to +100)
        motor_power = error_from_center * 200.0  # Scale to ±100
        motor_power = max(-100.0, min(100.0, motor_power))
        
        # Publish steering command
        steer_msg = SteerCommand()
        steer_msg.steer = motor_power / 100.0
        self.steer_pub.publish(steer_msg)
        
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                self.get_logger().info(
                    f"OPEN-LOOP: desired={self._desired_position:.3f}, "
                    f"power={motor_power:.1f}% (no sensor feedback)"
                )
                self.last_log_time = now
    
    def _power_to_direction_speed(self, power: float) -> tuple[int, int]:
        """Convert motor power to direction and speed.
        
        Args:
            power: Motor power in range -100 to +100
                  Negative = left, Positive = right
        
        Returns:
            Tuple of (direction, speed) where:
            direction: 0=stop, 1=left, 2=right
            speed: 0-100
        """
        if abs(power) < 1.0:  # Deadband
            return (0, 0)
        elif power > 0:
            return (2, min(100, int(abs(power))))  # Right
        else:
            return (1, min(100, int(abs(power))))  # Left
    
    def _publish_diagnostics(self, error: float, power: float) -> None:
        """Publish diagnostic information.
        
        Args:
            error: Current control error
            power: Current motor power output
        """
        msg = String()
        p, i, d = self.pid.get_terms()
        
        msg.data = (
            f"desired={self._desired_position:.3f}, "
            f"actual={self._actual_position:.3f}, "
            f"error={error:.3f}, "
            f"power={power:.1f}, "
            f"P={p:.2f}, I={i:.2f}, D={d:.2f}, "
            f"integral={self.pid.get_integral():.2f}, "
            f"sensor_ok={self._sensor_available}"
        )
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed."""
        if self.verbose:
            self.get_logger().info("Shutting down PID steering controller...")
        
        # Send stop command
        steer_msg = SteerCommand()
        steer_msg.steer = 0.0
        self.steer_pub.publish(steer_msg)
        
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node."""
    rclpy.init(args=args)
    
    node = PIDSteerControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""PID steering controller node for closed-loop steering control.

This node implements a PID (Proportional-Integral-Derivative) controller
that uses joystick commands as setpoints and pot meter feedback to compute
corrected steering commands for precise position control.

The node can operate in two modes:
1. PID enabled: Reads pot meter feedback and applies PID corrections
2. PID disabled (passthrough): Directly forwards joystick commands unchanged

This allows the system to work both with and without a functioning pot meter,
and enables easy switching between open-loop and closed-loop control.

Author: amnis_controller
Target: ROS2 systems with steering pot meter feedback
"""

from typing import Optional
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from amnis_controller.msg import JoystickCommand, SensorData


class PIDSteerControllerNode(Node):
    """ROS2 node for PID-based steering control with pot meter feedback.
    
    This node sits between the joystick normalizer and vehicle controller:
    - Subscribes to normalized joystick commands (JoystickCommand) as setpoints
    - Subscribes to sensor data (SensorData) for pot meter feedback
    - Applies PID algorithm to compute corrected steering values
    - Publishes modified JoystickCommand with PID-corrected steering
    - Passes through throttle, brake, gear, and cmd fields unchanged
    
    When PID is disabled, it acts as a simple passthrough, maintaining
    the current open-loop behavior until the pot meter is repaired.
    """

    def __init__(self) -> None:
        """Initialize the PID steering controller node."""
        super().__init__('pid_steer_controller')

        # ==================== Topic Configuration ====================
        self.declare_parameter('input_topic', 'normalized_joystick')
        self.declare_parameter('output_topic', 'steering_vehicle_controller_command')
        self.declare_parameter('feedback_topic', 'sensor_data')
        self.declare_parameter('diagnostic_topic', 'pid_diagnostics')
        
        # ==================== PID Gains ====================
        # Proportional gain: How strongly to react to current error
        # Higher values = faster response but more oscillation
        self.declare_parameter('kp', 1.5)
        
        # Integral gain: How strongly to react to accumulated error over time
        # Eliminates steady-state error but can cause overshoot
        self.declare_parameter('ki', 0.2)
        
        # Derivative gain: How strongly to react to rate of error change
        # Reduces overshoot and improves stability, but sensitive to noise
        self.declare_parameter('kd', 0.1)
        
        # ==================== PID Limits ====================
        # Maximum integral term to prevent windup (unbounded accumulation)
        self.declare_parameter('integral_limit', 0.5)
        
        # Maximum output value (should match steering command range)
        self.declare_parameter('output_limit', 1.0)
        
        # Maximum time to accumulate integral term (anti-windup)
        self.declare_parameter('max_integral_time_sec', 2.0)
        
        # ==================== Pot Meter Mapping ====================
        # These parameters map pot meter readings [0.0-1.0] to steering [-1.0, 1.0]
        # pot_center: Pot meter value when steering is centered (typically 0.5)
        # pot_min: Pot meter value at full left steering (maps to -1.0)
        # pot_max: Pot meter value at full right steering (maps to +1.0)
        self.declare_parameter('pot_min', 0.0)      # Full left position
        self.declare_parameter('pot_center', 0.5)   # Center position
        self.declare_parameter('pot_max', 1.0)      # Full right position
        
        # Deadzone around center to prevent jitter when centered
        self.declare_parameter('pot_deadzone', 0.02)
        
        # ==================== Control Parameters ====================
        # Enable/disable PID control (false = passthrough mode)
        self.declare_parameter('enable_pid', False)
        
        # Update rate in Hz (matches sensor_reader rate for consistency)
        self.declare_parameter('update_rate_hz', 10.0)
        
        # Timeout for feedback data - switch to passthrough if no sensor data
        self.declare_parameter('feedback_timeout_sec', 0.5)
        
        # Low-pass filter coefficient for derivative term (reduces noise)
        # Lower values = more filtering, slower response (range: 0.0-1.0)
        self.declare_parameter('derivative_filter_alpha', 0.1)
        
        # ==================== Testing & Diagnostics ====================
        # Mock mode: Simulate pot meter feedback for testing without hardware
        self.declare_parameter('mock_mode', False)
        
        # Mock mode parameters: First-order lag simulation
        self.declare_parameter('mock_time_constant', 0.3)  # Response speed
        
        # Enable verbose logging
        self.declare_parameter('verbose', True)
        
        # Throttle logging frequency
        self.declare_parameter('log_throttle_sec', 1.0)
        
        # Publish diagnostic messages
        self.declare_parameter('publish_diagnostics', True)
        
        # ==================== Get Parameter Values ====================
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        
        self.integral_limit = self.get_parameter('integral_limit').value
        self.output_limit = self.get_parameter('output_limit').value
        self.max_integral_time = self.get_parameter('max_integral_time_sec').value
        
        self.pot_min = self.get_parameter('pot_min').value
        self.pot_center = self.get_parameter('pot_center').value
        self.pot_max = self.get_parameter('pot_max').value
        self.pot_deadzone = self.get_parameter('pot_deadzone').value
        
        self.enable_pid = self.get_parameter('enable_pid').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.feedback_timeout = self.get_parameter('feedback_timeout_sec').value
        self.derivative_alpha = self.get_parameter('derivative_filter_alpha').value
        
        self.mock_mode = self.get_parameter('mock_mode').value
        self.mock_time_constant = self.get_parameter('mock_time_constant').value
        
        self.verbose = self.get_parameter('verbose').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        
        # ==================== PID State Variables ====================
        # Current setpoint from joystick (desired steering position)
        self._setpoint: float = 0.0
        
        # Current feedback from pot meter (actual steering position)
        self._feedback: float = 0.0
        self._feedback_raw: Optional[float] = None  # Raw pot meter value [0,1]
        
        # PID error tracking
        self._error: float = 0.0
        self._last_error: float = 0.0
        self._integral: float = 0.0
        self._derivative: float = 0.0
        self._filtered_derivative: float = 0.0
        
        # Individual PID term contributions (for diagnostics)
        self._p_term: float = 0.0
        self._i_term: float = 0.0
        self._d_term: float = 0.0
        
        # Timing for integral windup protection
        self._integral_start_time: Optional[float] = None
        
        # Last update time for derivative calculation
        self._last_update_time: Optional[float] = None
        
        # ==================== Feedback Status ====================
        # Last time we received feedback data
        self._last_feedback_time: Optional[float] = None
        
        # Pot meter calibration status from SensorData
        self._pot_calibrated: bool = False
        
        # Track if we're in passthrough mode and why
        self._passthrough_reason: Optional[str] = None
        
        # ==================== Mock Mode State ====================
        # Simulated pot meter position for testing
        self._mock_position: float = 0.5  # Start at center
        
        # ==================== Last Received Command ====================
        # Store last joystick command for passthrough/processing
        self._last_joystick_cmd: Optional[JoystickCommand] = None
        
        # ==================== Logging ====================
        self.last_log_time = self.get_clock().now()
        
        # ==================== Create Subscribers ====================
        # Subscribe to normalized joystick commands (setpoints)
        self.joystick_sub = self.create_subscription(
            JoystickCommand,
            input_topic,
            self.joystick_callback,
            10
        )
        
        # Subscribe to sensor data (pot meter feedback)
        self.sensor_sub = self.create_subscription(
            SensorData,
            feedback_topic,
            self.sensor_callback,
            10
        )
        
        # ==================== Create Publishers ====================
        # Publish PID-corrected commands to vehicle controller
        self.output_pub = self.create_publisher(
            JoystickCommand,
            output_topic,
            10
        )
        
        # Publish diagnostics for monitoring and tuning
        if self.publish_diagnostics:
            self.diagnostic_pub = self.create_publisher(
                String,
                diagnostic_topic,
                10
            )
        
        # ==================== Create Update Timer ====================
        # Periodic callback for PID computation
        update_period = 1.0 / update_rate
        self.update_timer = self.create_timer(
            update_period,
            self.update_callback
        )
        
        # ==================== Initialization Complete ====================
        mode_str = "PID ENABLED" if self.enable_pid else "PASSTHROUGH (PID disabled)"
        mock_str = " [MOCK MODE]" if self.mock_mode else ""
        
        if self.verbose:
            self.get_logger().info(
                f"PID Steering Controller initialized: {mode_str}{mock_str}\n"
                f"  Topics: {input_topic} -> {output_topic}\n"
                f"  Feedback: {feedback_topic}\n"
                f"  PID Gains: Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}\n"
                f"  Pot Mapping: [{self.pot_min:.2f}, {self.pot_center:.2f}, {self.pot_max:.2f}]\n"
                f"  Update Rate: {update_rate:.1f} Hz"
            )
    
    def joystick_callback(self, msg: JoystickCommand) -> None:
        """Callback for joystick commands (setpoints).
        
        Stores the joystick command for processing in the update callback.
        The steering value from the joystick becomes our PID setpoint.
        
        Args:
            msg: JoystickCommand with throttle, steer, gear, brake, cmd fields
        """
        # Store the entire command for later processing
        self._last_joystick_cmd = msg
        
        # Extract steering setpoint (desired position)
        self._setpoint = msg.steer
    
    def sensor_callback(self, msg: SensorData) -> None:
        """Callback for sensor data (pot meter feedback).
        
        Receives pot meter readings and updates feedback state.
        The steering_wheel field (0.0-1.0) represents the actual
        position of the steering wheel.
        
        Args:
            msg: SensorData with gas_pedal, steering_wheel, and calibration info
        """
        # Record when we received feedback
        self._last_feedback_time = time.time()
        
        # Store raw pot meter value
        self._feedback_raw = msg.steering_wheel
        
        # Store calibration status
        self._pot_calibrated = msg.calibrated
        
        # Map pot meter reading [0,1] to steering range [-1,1]
        self._feedback = self._map_pot_to_steering(msg.steering_wheel)
    
    def _map_pot_to_steering(self, pot_value: float) -> float:
        """Map pot meter reading to steering command range.
        
        Converts pot meter value [0.0-1.0] to steering command [-1.0, 1.0]
        using configurable min/center/max parameters.
        
        The mapping handles asymmetric ranges (left and right may have
        different spans) and applies a deadzone around center.
        
        Args:
            pot_value: Pot meter reading in range [0.0, 1.0]
            
        Returns:
            Steering value in range [-1.0, 1.0]
        """
        # Check for center deadzone to prevent jitter
        if abs(pot_value - self.pot_center) < self.pot_deadzone:
            return 0.0
        
        # Map left side: pot_min to pot_center -> -1.0 to 0.0
        if pot_value < self.pot_center:
            # Calculate position in left range
            left_range = self.pot_center - self.pot_min
            if left_range > 0:
                # Normalize to [0,1] in left range, then scale to [-1,0]
                normalized = (pot_value - self.pot_min) / left_range
                return normalized - 1.0  # Maps [0,1] to [-1,0]
            else:
                return -1.0  # Degenerate case: no left range
        
        # Map right side: pot_center to pot_max -> 0.0 to 1.0
        else:
            # Calculate position in right range
            right_range = self.pot_max - self.pot_center
            if right_range > 0:
                # Normalize to [0,1] in right range
                normalized = (pot_value - self.pot_center) / right_range
                return normalized  # Maps [0,1] to [0,1]
            else:
                return 1.0  # Degenerate case: no right range
    
    def _update_mock_feedback(self, dt: float) -> None:
        """Simulate pot meter feedback for testing without hardware.
        
        Implements a first-order lag (low-pass filter) to simulate
        realistic steering dynamics. The mock position slowly follows
        the commanded output.
        
        Args:
            dt: Time step since last update (seconds)
        """
        if not self.mock_mode:
            return
        
        # Use the setpoint as the target position for simulation
        # In reality, the pot meter follows the actual motor output,
        # but for simple testing we can use setpoint directly
        target = (self._setpoint + 1.0) / 2.0  # Map [-1,1] to [0,1]
        
        # First-order lag: position moves toward target exponentially
        # tau = time constant, alpha = 1 - exp(-dt/tau)
        if self.mock_time_constant > 0:
            alpha = 1.0 - pow(2.71828, -dt / self.mock_time_constant)
            self._mock_position += alpha * (target - self._mock_position)
        else:
            # No lag - instant response
            self._mock_position = target
        
        # Update feedback with simulated value
        self._feedback_raw = self._mock_position
        self._feedback = self._map_pot_to_steering(self._mock_position)
        self._last_feedback_time = time.time()
    
    def _check_feedback_status(self) -> bool:
        """Check if feedback data is fresh and valid.
        
        Returns:
            True if feedback is available and fresh, False otherwise
        """
        # In mock mode, feedback is always "available"
        if self.mock_mode:
            return True
        
        # Check if we've ever received feedback
        if self._last_feedback_time is None:
            self._passthrough_reason = "No feedback received yet"
            return False
        
        # Check if feedback is stale (timeout)
        time_since_feedback = time.time() - self._last_feedback_time
        if time_since_feedback > self.feedback_timeout:
            self._passthrough_reason = f"Feedback timeout ({time_since_feedback:.2f}s)"
            return False
        
        # Check if pot meter value is valid (within expected range)
        if self._feedback_raw is not None:
            if self._feedback_raw < 0.0 or self._feedback_raw > 1.0:
                self._passthrough_reason = f"Invalid pot value ({self._feedback_raw:.3f})"
                return False
        
        # Feedback is valid
        self._passthrough_reason = None
        return True
    
    def _compute_pid(self, dt: float) -> float:
        """Compute PID output based on error and update PID state.
        
        Implements a PID controller with:
        - Proportional term: reacts to current error
        - Integral term: eliminates steady-state error
        - Derivative term: improves stability and reduces overshoot
        
        Includes anti-windup protection and derivative filtering.
        
        Args:
            dt: Time step since last update (seconds)
            
        Returns:
            PID output (steering correction) in range [-output_limit, output_limit]
        """
        # Calculate error (setpoint - feedback)
        # Positive error means we need to steer right
        # Negative error means we need to steer left
        self._error = self._setpoint - self._feedback
        
        # ==================== Proportional Term ====================
        # Proportional term is simply Kp * error
        self._p_term = self.kp * self._error
        
        # ==================== Integral Term ====================
        # Accumulate error over time to eliminate steady-state error
        # Only accumulate if dt is valid and we haven't exceeded time limit
        if dt > 0:
            # Check if we should reset integral (anti-windup)
            current_time = time.time()
            
            # Start tracking integral accumulation time if not started
            if self._integral_start_time is None:
                self._integral_start_time = current_time
            
            # Reset integral if it's been accumulating too long
            time_accumulating = current_time - self._integral_start_time
            if time_accumulating > self.max_integral_time:
                self._integral = 0.0
                self._integral_start_time = current_time
            
            # Accumulate integral (trapezoidal integration)
            self._integral += self._error * dt
            
            # Clamp integral to prevent windup
            if self._integral > self.integral_limit:
                self._integral = self.integral_limit
            elif self._integral < -self.integral_limit:
                self._integral = -self.integral_limit
        
        self._i_term = self.ki * self._integral
        
        # ==================== Derivative Term ====================
        # Calculate rate of change of error
        # NOTE: We differentiate the FEEDBACK, not the error, to prevent
        # "derivative kick" when setpoint changes suddenly
        if dt > 0 and self._last_error is not None:
            # Calculate raw derivative (change in error / time)
            self._derivative = (self._error - self._last_error) / dt
            
            # Apply low-pass filter to reduce noise
            # filtered = alpha * new + (1 - alpha) * old
            self._filtered_derivative = (
                self.derivative_alpha * self._derivative +
                (1.0 - self.derivative_alpha) * self._filtered_derivative
            )
        
        self._d_term = self.kd * self._filtered_derivative
        
        # ==================== Compute Total Output ====================
        output = self._p_term + self._i_term + self._d_term
        
        # Clamp output to limits
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
        
        # Update last error for next derivative calculation
        self._last_error = self._error
        
        return output
    
    def update_callback(self) -> None:
        """Periodic callback for PID computation and command publishing.
        
        This is the main control loop that:
        1. Checks if we have a joystick command to process
        2. Updates mock feedback if in mock mode
        3. Decides whether to use PID or passthrough mode
        4. Computes PID correction if enabled and feedback is valid
        5. Publishes the output command
        6. Publishes diagnostics
        """
        # Check if we have a joystick command to process
        if self._last_joystick_cmd is None:
            return
        
        # ==================== Calculate Time Step ====================
        current_time = time.time()
        if self._last_update_time is not None:
            dt = current_time - self._last_update_time
        else:
            dt = 0.0
        self._last_update_time = current_time
        
        # ==================== Update Mock Feedback ====================
        if self.mock_mode:
            self._update_mock_feedback(dt)
        
        # ==================== Determine Control Mode ====================
        # Check if PID should be active
        feedback_valid = self._check_feedback_status()
        use_pid = self.enable_pid and feedback_valid
        
        # ==================== Create Output Command ====================
        # Start with a copy of the input command
        output_cmd = JoystickCommand()
        output_cmd.throttle = self._last_joystick_cmd.throttle
        output_cmd.gear = self._last_joystick_cmd.gear
        output_cmd.brake = self._last_joystick_cmd.brake
        output_cmd.cmd = self._last_joystick_cmd.cmd
        
        # ==================== Compute Steering Output ====================
        if use_pid:
            # PID mode: Compute correction and apply it
            pid_output = self._compute_pid(dt)
            output_cmd.steer = pid_output
            
            # Update passthrough reason
            self._passthrough_reason = None
        else:
            # Passthrough mode: Forward joystick steering unchanged
            output_cmd.steer = self._last_joystick_cmd.steer
            
            # Reset PID state when not active
            self._integral = 0.0
            self._filtered_derivative = 0.0
            self._integral_start_time = None
            
            # Set passthrough reason if not already set
            if self._passthrough_reason is None:
                if not self.enable_pid:
                    self._passthrough_reason = "PID disabled"
                else:
                    self._passthrough_reason = "Unknown"
        
        # ==================== Publish Output ====================
        self.output_pub.publish(output_cmd)
        
        # ==================== Periodic Logging ====================
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                if use_pid:
                    self.get_logger().info(
                        f"PID Active: "
                        f"setpoint={self._setpoint:.3f}, "
                        f"feedback={self._feedback:.3f} (raw={self._feedback_raw:.3f}), "
                        f"error={self._error:.3f}, "
                        f"P={self._p_term:.3f}, I={self._i_term:.3f}, D={self._d_term:.3f}, "
                        f"output={output_cmd.steer:.3f}, "
                        f"cal={self._pot_calibrated}"
                    )
                else:
                    self.get_logger().info(
                        f"Passthrough: "
                        f"steer={output_cmd.steer:.3f}, "
                        f"reason='{self._passthrough_reason}'"
                    )
                self.last_log_time = now
        
        # ==================== Publish Diagnostics ====================
        if self.publish_diagnostics:
            self._publish_diagnostics(output_cmd.steer, use_pid)
    
    def _publish_diagnostics(self, output: float, pid_active: bool) -> None:
        """Publish diagnostic information for monitoring and tuning.
        
        Args:
            output: Computed output value
            pid_active: Whether PID is currently active
        """
        msg = String()
        
        # Build comprehensive diagnostic string
        diag_data = (
            f"setpoint={self._setpoint:.3f}, "
            f"feedback={self._feedback:.3f}, "
            f"feedback_raw={self._feedback_raw if self._feedback_raw is not None else 'None'}, "
            f"error={self._error:.3f}, "
            f"P={self._p_term:.3f}, "
            f"I={self._i_term:.3f} (integral={self._integral:.3f}), "
            f"D={self._d_term:.3f} (filtered_deriv={self._filtered_derivative:.3f}), "
            f"output={output:.3f}, "
            f"pid_active={pid_active}, "
            f"pid_enabled={self.enable_pid}, "
            f"passthrough_reason='{self._passthrough_reason if self._passthrough_reason else 'N/A'}', "
            f"pot_calibrated={self._pot_calibrated}, "
            f"mock_mode={self.mock_mode}"
        )
        
        msg.data = diag_data
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed.
        
        Returns:
            True if cleanup was successful
        """
        if self.verbose:
            self.get_logger().info("Shutting down PID steering controller...")
        
        return super().destroy_node()


def main(args=None) -> None:
    """Entry point for the PID steering controller node.
    
    This function is called when you run:
        ros2 run amnis_controller pid_steer_controller_node
    
    It:
    1. Initializes ROS 2 (rclpy.init)
    2. Creates an instance of the PID controller node
    3. Spins the node (runs the main loop that processes callbacks)
    4. Cleans up when interrupted
    
    Args:
        args: Command line arguments (passed to rclpy.init)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create node instance
    node = PIDSteerControllerNode()
    
    try:
        # Spin keeps the node running and processing callbacks
        # It will block here until Ctrl+C or rclpy.shutdown()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        # Always clean up
        node.destroy_node()
        
        # Only shutdown if rclpy is still initialized
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

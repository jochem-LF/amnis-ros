#!/usr/bin/env python3
"""Sensor reader node for reading analog sensors via ADS1015L ADC.

This node reads gas pedal and steering wheel potentiometers from an ADS1015L
ADC via I2C and publishes the normalized values to ROS2 topics.

Author: amnis_controller
Target: Systems with I2C ADC (ADS1015L)
"""

from typing import Sequence, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from amnis_controller.msg import SensorData

from amnis_controller.drivers import ADCDriver


class SensorReaderNode(Node):
    """ROS2 node for reading analog sensors via ADC.
    
    This node:
    - Reads gas pedal (AIN0) and steering wheel (AIN1) from ADS1015L ADC
    - Publishes normalized values (0.0-1.0) to /sensor_data topic
    - Supports dynamic calibration or preset calibration values
    - Publishes diagnostics
    - Implements error handling and reconnection
    """

    def __init__(self) -> None:
        """Initialize the sensor reader node."""
        super().__init__('sensor_reader')

        # Declare parameters
        self.declare_parameter('output_topic', 'sensor_data')
        self.declare_parameter('diagnostic_topic', 'sensor_diagnostics')
        
        # Hardware configuration
        self.declare_parameter('i2c_bus', 1)  # I2C bus 1 on Raspberry Pi
        self.declare_parameter('i2c_address', 0x48)  # ADS1015L default address
        self.declare_parameter('mock_mode', False)  # For testing without hardware
        
        # Pigpio connection configuration (for remote Raspberry Pi)
        self.declare_parameter('pigpio_host', 'localhost')
        self.declare_parameter('pigpio_port', 8888)
        
        # Calibration parameters
        self.declare_parameter('gas_pedal_min', 0)  # Raw ADC min value
        self.declare_parameter('gas_pedal_max', 2047)  # Raw ADC max value
        self.declare_parameter('steering_wheel_min', 0)  # Raw ADC min value
        self.declare_parameter('steering_wheel_max', 2047)  # Raw ADC max value
        self.declare_parameter('auto_calibrate', False)  # Start in calibration mode
        self.declare_parameter('calibration_duration_sec', 10.0)  # Auto-calibration duration
        
        # Control parameters
        self.declare_parameter('update_rate_hz', 50.0)  # Sensor reading rate (50 Hz)
        
        # Diagnostics
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)  # Enable/disable info logging
        
        # Get parameter values
        output_topic = self.get_parameter('output_topic').value
        diagnostic_topic = self.get_parameter('diagnostic_topic').value
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        mock_mode = self.get_parameter('mock_mode').value
        pigpio_host = self.get_parameter('pigpio_host').value
        pigpio_port = self.get_parameter('pigpio_port').value
        gas_min = self.get_parameter('gas_pedal_min').value
        gas_max = self.get_parameter('gas_pedal_max').value
        steer_min = self.get_parameter('steering_wheel_min').value
        steer_max = self.get_parameter('steering_wheel_max').value
        self.auto_calibrate = self.get_parameter('auto_calibrate').value
        self.calibration_duration = self.get_parameter('calibration_duration_sec').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.verbose = self.get_parameter('verbose').value
        
        # Check if custom calibration values are provided
        has_calibration = (
            gas_min != 0 or gas_max != 2047 or 
            steer_min != 0 or steer_max != 2047
        )
        
        # Initialize hardware driver
        self.driver = ADCDriver(
            i2c_bus=i2c_bus,
            i2c_address=i2c_address,
            mock_mode=mock_mode,
            pigpio_host=pigpio_host,
            pigpio_port=pigpio_port,
            gas_min=gas_min if has_calibration else None,
            gas_max=gas_max if has_calibration else None,
            steer_min=steer_min if has_calibration else None,
            steer_max=steer_max if has_calibration else None,
        )
        
        if not self.driver.is_connected():
            self.get_logger().error(
                "Failed to initialize ADC driver! Running in degraded mode."
            )
        
        # Calibration state
        self._calibration_start_time = None
        self._is_calibrated = has_calibration
        
        # Start auto-calibration if requested
        if self.auto_calibrate and not has_calibration:
            self.get_logger().info(
                f"Starting auto-calibration for {self.calibration_duration:.1f} seconds. "
                "Move gas pedal and steering wheel through full range!"
            )
            self.driver.start_calibration()
            self._calibration_start_time = self.get_clock().now()
        
        # State tracking
        self._last_gas_value: Optional[float] = None
        self._last_steer_value: Optional[float] = None
        self._read_error_count = 0
        
        # Create publisher
        self.publisher = self.create_publisher(
            SensorData,
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
        
        # Create update timer for periodic sensor reading
        update_period = 1.0 / update_rate
        self.update_timer = self.create_timer(
            update_period,
            self.update_callback
        )
        
        # Logging timer
        self.last_log_time = self.get_clock().now()
        
        if self.verbose:
            self.get_logger().info(
                f"Sensor reader initialized: "
                f"topic={output_topic}, "
                f"i2c_bus={i2c_bus}, "
                f"i2c_address=0x{i2c_address:02X}, "
                f"rate={update_rate}Hz, "
                f"pigpio_host={pigpio_host}, "
                f"calibrated={self._is_calibrated}, "
                f"mock={mock_mode}"
            )
    
    def update_callback(self) -> None:
        """Periodic callback to read sensors and publish data."""
        
        # Check if auto-calibration should end
        if self.driver.is_calibrating() and self._calibration_start_time is not None:
            elapsed = (self.get_clock().now() - self._calibration_start_time).nanoseconds / 1e9
            if elapsed >= self.calibration_duration:
                success = self.driver.end_calibration()
                if success:
                    self._is_calibrated = True
                    gas_min, gas_max, steer_min, steer_max = self.driver.get_calibration()
                    self.get_logger().info(
                        f"Auto-calibration complete! "
                        f"Gas: {gas_min}-{gas_max}, Steering: {steer_min}-{steer_max}"
                    )
                else:
                    self.get_logger().error("Auto-calibration failed!")
                self._calibration_start_time = None
        
        # Read sensor values
        gas_value = self.driver.get_gas_pedal()
        steer_value = self.driver.get_steering_wheel()
        
        # Read raw values for diagnostics
        gas_raw = self.driver.read_raw(self.driver.CHANNEL_GAS_PEDAL)
        steer_raw = self.driver.read_raw(self.driver.CHANNEL_STEERING_WHEEL)
        
        # Check for read errors
        if gas_value is None or steer_value is None:
            self._read_error_count += 1
            if self.verbose:
                self.get_logger().warning(
                    f"Failed to read sensors (error count: {self._read_error_count})"
                )
            return
        
        # Store values
        self._last_gas_value = gas_value
        self._last_steer_value = steer_value
        
        # Create and publish message
        msg = SensorData()
        msg.gas_pedal = gas_value
        msg.steering_wheel = steer_value
        msg.gas_pedal_raw = gas_raw if gas_raw is not None else -1
        msg.steering_wheel_raw = steer_raw if steer_raw is not None else -1
        msg.calibrated = self._is_calibrated
        
        self.publisher.publish(msg)
        
        # Periodic logging
        if self.verbose:
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds / 1e9 >= self.log_throttle:
                calib_status = "CALIBRATED" if self._is_calibrated else "UNCALIBRATED"
                if self.driver.is_calibrating():
                    calib_status = "CALIBRATING"
                
                self.get_logger().info(
                    f"Sensors [{calib_status}]: "
                    f"gas={gas_value:.3f} (raw={gas_raw}), "
                    f"steer={steer_value:.3f} (raw={steer_raw}) | "
                    f"Connected: {self.driver.is_connected()} | "
                    f"Errors: {self.driver.get_error_count()}"
                )
                self.last_log_time = now
        
        # Publish diagnostics
        if self.publish_diagnostics:
            self._publish_diagnostics(gas_value, steer_value, gas_raw, steer_raw)
    
    def _publish_diagnostics(
        self, 
        gas_value: float, 
        steer_value: float,
        gas_raw: Optional[int],
        steer_raw: Optional[int]
    ) -> None:
        """Publish diagnostic information.
        
        Args:
            gas_value: Normalized gas pedal value
            steer_value: Normalized steering wheel value
            gas_raw: Raw gas pedal ADC value
            steer_raw: Raw steering wheel ADC value
        """
        msg = String()
        
        gas_min, gas_max, steer_min, steer_max = self.driver.get_calibration()
        
        diag_data = (
            f"gas={gas_value:.3f}, "
            f"steer={steer_value:.3f}, "
            f"gas_raw={gas_raw}, "
            f"steer_raw={steer_raw}, "
            f"calibrated={self._is_calibrated}, "
            f"calibrating={self.driver.is_calibrating()}, "
            f"gas_cal={gas_min}-{gas_max}, "
            f"steer_cal={steer_min}-{steer_max}, "
            f"connected={self.driver.is_connected()}, "
            f"errors={self.driver.get_error_count()}, "
            f"read_errors={self._read_error_count}"
        )
        
        msg.data = diag_data
        self.diagnostic_pub.publish(msg)
    
    def destroy_node(self) -> bool:
        """Cleanup when node is destroyed.
        
        Ensures ADC connection is properly cleaned up.
        """
        if self.verbose:
            self.get_logger().info("Shutting down sensor reader...")
        
        # Close hardware connection
        self.driver.close()
        
        # Call parent cleanup
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node when run as a standalone executable.
    
    This function is called when you run:
        ros2 run amnis_controller sensor_reader_node
    
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
    node = SensorReaderNode()
    
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

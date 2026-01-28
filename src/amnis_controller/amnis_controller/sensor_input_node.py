#!/usr/bin/env python3
"""Sensor input node for reading analog sensors via ADS1015L ADC.

This module implements a ROS 2 node that:
1. Reads analog sensor values from an ADS1015L ADC via I2C
2. Normalizes the voltage readings to standard ranges
3. Publishes sensor values on separate topics for downstream consumption

The node reads two potentiometers:
- Gas pedal on AIN0: normalized to [0.0, 1.0] (no pedal to full press)
- Steering wheel on AIN1: normalized to [-1.0, 1.0] (left to right)

This provides a parallel input method to the joystick, allowing the vehicle
to be controlled via physical pedals and steering wheel sensors.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from amnis_controller.drivers.adc_driver import ADCDriver


class SensorInputNode(Node):
    """ROS 2 node that reads analog sensors and publishes normalized values.
    
    This node:
    - Reads gas pedal and steering wheel potentiometers via ADC
    - Normalizes voltage readings to standard ranges
    - Publishes Float32 messages on /sensor/gas_pedal and /sensor/steering_wheel
    - Supports configurable calibration parameters for each sensor
    - Runs at configurable update rate (default 20Hz)
    """

    def __init__(self) -> None:
        """Initialize the sensor input node.
        
        Sets up ROS parameters, creates the ADC driver, creates publishers,
        and starts a timer to periodically read and publish sensor values.
        """
        super().__init__('sensor_input')

        # Declare ADC configuration parameters
        self.declare_parameter('adc_i2c_bus', 1)
        self.declare_parameter('adc_i2c_address', 0x48)
        
        # Declare channel parameters
        self.declare_parameter('gas_pedal_channel', 0)  # AIN0
        self.declare_parameter('steering_channel', 1)    # AIN1
        
        # Declare calibration parameters (MUST be calibrated by user)
        # These default values are placeholders - measure actual min/max voltages!
        self.declare_parameter('gas_min_voltage', 0.5)
        self.declare_parameter('gas_max_voltage', 3.0)
        self.declare_parameter('steer_min_voltage', 0.3)
        self.declare_parameter('steer_max_voltage', 3.2)
        
        # Declare operational parameters
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('pigpio_host', '192.168.10.2')
        self.declare_parameter('pigpio_port', 8888)
        self.declare_parameter('mock_mode', False)
        
        # Declare logging parameters
        self.declare_parameter('log_throttle_sec', 1.0)
        self.declare_parameter('verbose', True)
        
        # Declare topic parameters
        self.declare_parameter('gas_pedal_topic', '/sensor/gas_pedal')
        self.declare_parameter('steering_topic', '/sensor/steering_wheel')
        
        # Retrieve parameters
        adc_i2c_bus = int(self.get_parameter('adc_i2c_bus').value)
        adc_i2c_address = int(self.get_parameter('adc_i2c_address').value)
        
        self._gas_channel = int(self.get_parameter('gas_pedal_channel').value)
        self._steering_channel = int(self.get_parameter('steering_channel').value)
        
        self._gas_min_voltage = float(self.get_parameter('gas_min_voltage').value)
        self._gas_max_voltage = float(self.get_parameter('gas_max_voltage').value)
        self._steer_min_voltage = float(self.get_parameter('steer_min_voltage').value)
        self._steer_max_voltage = float(self.get_parameter('steer_max_voltage').value)
        
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        pigpio_host = str(self.get_parameter('pigpio_host').value)
        pigpio_port = int(self.get_parameter('pigpio_port').value)
        mock_mode = bool(self.get_parameter('mock_mode').value)
        
        self._log_throttle_sec = max(float(self.get_parameter('log_throttle_sec').value), 0.0)
        self.verbose = bool(self.get_parameter('verbose').value)
        
        gas_pedal_topic = str(self.get_parameter('gas_pedal_topic').value)
        steering_topic = str(self.get_parameter('steering_topic').value)
        
        # Validate parameters
        if self._gas_min_voltage >= self._gas_max_voltage:
            self.get_logger().error(
                f"Invalid gas pedal calibration: min={self._gas_min_voltage}V >= max={self._gas_max_voltage}V"
            )
            raise ValueError("gas_min_voltage must be less than gas_max_voltage")
        
        if self._steer_min_voltage >= self._steer_max_voltage:
            self.get_logger().error(
                f"Invalid steering calibration: min={self._steer_min_voltage}V >= max={self._steer_max_voltage}V"
            )
            raise ValueError("steer_min_voltage must be less than steer_max_voltage")
        
        if update_rate_hz <= 0:
            self.get_logger().error(f"Invalid update rate: {update_rate_hz}Hz (must be > 0)")
            raise ValueError("update_rate_hz must be positive")
        
        # Initialize ADC driver
        try:
            self._adc = ADCDriver(
                i2c_bus=adc_i2c_bus,
                i2c_address=adc_i2c_address,
                mock_mode=mock_mode,
                pigpio_host=pigpio_host,
                pigpio_port=pigpio_port,
            )
            
            if not self._adc.is_connected():
                self.get_logger().error("Failed to connect to ADC")
                raise RuntimeError("ADC connection failed")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ADC driver: {e}")
            raise
        
        # Create publishers
        self._gas_publisher = self.create_publisher(Float32, gas_pedal_topic, 10)
        self._steering_publisher = self.create_publisher(Float32, steering_topic, 10)
        
        # Create timer for periodic sensor reading
        timer_period = 1.0 / update_rate_hz  # seconds
        self._timer = self.create_timer(timer_period, self._timer_callback)
        
        # Track error counts
        self._consecutive_errors = 0
        self._max_consecutive_errors = 10
        
        # Log startup information
        if self.verbose:
            self.get_logger().info(
                f"Sensor input node ready:\n"
                f"  ADC: bus={adc_i2c_bus}, address=0x{adc_i2c_address:02x}, host={pigpio_host}\n"
                f"  Gas pedal: channel={self._gas_channel}, range={self._gas_min_voltage:.3f}V-{self._gas_max_voltage:.3f}V → [0.0, 1.0]\n"
                f"  Steering: channel={self._steering_channel}, range={self._steer_min_voltage:.3f}V-{self._steer_max_voltage:.3f}V → [-1.0, 1.0]\n"
                f"  Update rate: {update_rate_hz:.1f}Hz"
            )
            
            if mock_mode:
                self.get_logger().warn("Running in MOCK mode - no actual hardware communication")
    
    def _normalize_sensor(
        self,
        voltage: float,
        min_voltage: float,
        max_voltage: float,
        output_min: float,
        output_max: float
    ) -> float:
        """Normalize sensor voltage to output range.
        
        Performs linear mapping from input voltage range to output range,
        with clamping to ensure output stays within bounds.
        
        Args:
            voltage: Measured voltage from sensor
            min_voltage: Minimum expected voltage (calibrated)
            max_voltage: Maximum expected voltage (calibrated)
            output_min: Minimum output value
            output_max: Maximum output value
        
        Returns:
            Normalized value in [output_min, output_max] range
        """
        # Clamp input to calibrated range
        voltage = max(min_voltage, min(max_voltage, voltage))
        
        # Linear mapping: normalize to [0, 1]
        normalized = (voltage - min_voltage) / (max_voltage - min_voltage)
        
        # Scale to output range
        output = output_min + normalized * (output_max - output_min)
        
        # Final clamp (safety)
        return max(output_min, min(output_max, output))
    
    def _timer_callback(self) -> None:
        """Timer callback to read sensors and publish values.
        
        This is called periodically (at update_rate_hz) to:
        1. Read raw voltage from gas pedal ADC channel
        2. Read raw voltage from steering wheel ADC channel
        3. Normalize both values to their respective ranges
        4. Publish normalized values on their topics
        5. Log readings if verbose mode is enabled
        """
        # Read gas pedal
        gas_voltage = self._adc.read_voltage(self._gas_channel)
        
        if gas_voltage is None:
            self._consecutive_errors += 1
            self.get_logger().error(
                f"Failed to read gas pedal (channel {self._gas_channel}), "
                f"consecutive errors: {self._consecutive_errors}"
            )
            
            if self._consecutive_errors >= self._max_consecutive_errors:
                self.get_logger().error(
                    f"Too many consecutive errors ({self._consecutive_errors}), "
                    "attempting to reconnect ADC..."
                )
                # Try to reinitialize
                self._adc._initialize_i2c()
                self._consecutive_errors = 0
            
            return
        
        # Read steering wheel
        steering_voltage = self._adc.read_voltage(self._steering_channel)
        
        if steering_voltage is None:
            self._consecutive_errors += 1
            self.get_logger().error(
                f"Failed to read steering wheel (channel {self._steering_channel}), "
                f"consecutive errors: {self._consecutive_errors}"
            )
            
            if self._consecutive_errors >= self._max_consecutive_errors:
                self.get_logger().error(
                    f"Too many consecutive errors ({self._consecutive_errors}), "
                    "attempting to reconnect ADC..."
                )
                # Try to reinitialize
                self._adc._initialize_i2c()
                self._consecutive_errors = 0
            
            return
        
        # Reset error counter on successful read
        self._consecutive_errors = 0
        
        # Normalize gas pedal: voltage → [0.0, 1.0]
        gas_normalized = self._normalize_sensor(
            gas_voltage,
            self._gas_min_voltage,
            self._gas_max_voltage,
            0.0,
            1.0
        )
        
        # Normalize steering: voltage → [-1.0, 1.0]
        steering_normalized = self._normalize_sensor(
            steering_voltage,
            self._steer_min_voltage,
            self._steer_max_voltage,
            -1.0,
            1.0
        )
        
        # Publish gas pedal
        gas_msg = Float32()
        gas_msg.data = gas_normalized
        self._gas_publisher.publish(gas_msg)
        
        # Publish steering
        steering_msg = Float32()
        steering_msg.data = steering_normalized
        self._steering_publisher.publish(steering_msg)
        
        # Log values (throttled)
        if self.verbose:
            self.get_logger().info(
                f"Gas: {gas_voltage:.3f}V → {gas_normalized:.3f} | "
                f"Steering: {steering_voltage:.3f}V → {steering_normalized:.3f}",
                throttle_duration_sec=self._log_throttle_sec
            )
    
    def destroy_node(self) -> bool:
        """Clean up resources when the node shuts down.
        
        Stops the timer, closes the ADC driver, and destroys publishers.
        
        Returns:
            True if cleanup was successful
        """
        # Stop the timer
        if hasattr(self, '_timer') and self._timer is not None:
            self._timer.cancel()
            self._timer = None
        
        # Close ADC driver
        if hasattr(self, '_adc') and self._adc is not None:
            try:
                self._adc.close()
                self.get_logger().info("ADC driver closed")
            except Exception as e:
                self.get_logger().error(f"Error closing ADC driver: {e}")
        
        # Destroy publishers
        if hasattr(self, '_gas_publisher') and self._gas_publisher is not None:
            self.destroy_publisher(self._gas_publisher)
            self._gas_publisher = None
        
        if hasattr(self, '_steering_publisher') and self._steering_publisher is not None:
            self.destroy_publisher(self._steering_publisher)
            self._steering_publisher = None
        
        # Call parent cleanup
        return super().destroy_node()


def main(args=None) -> None:
    """Entry point for the sensor input node.
    
    This function is called when you run:
        ros2 run amnis_controller sensor_input_node
    
    It initializes ROS 2, creates the node instance, runs the main loop,
    and handles graceful shutdown.
    
    Args:
        args: Command-line arguments passed to ROS 2
    """
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create node instance
    node = None
    try:
        node = SensorInputNode()
        
        # Run the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Unexpected error: {e}")
        raise
    finally:
        # Clean up
        if node is not None:
            node.destroy_node()
        
        # Shutdown ROS 2
        rclpy.shutdown()


if __name__ == '__main__':
    main()

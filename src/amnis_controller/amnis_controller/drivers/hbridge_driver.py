"""H-Bridge motor driver for steering control via remote pigpio I2C.

This module provides a hardware abstraction layer for controlling an H-bridge
motor driver over I2C through a remote Raspberry Pi running pigpio daemon.
It handles low-level communication, power/direction conversion, and error handling.

Compatible with any system that can connect to a pigpiod daemon over network.
"""

from typing import Optional
import logging
from .pigpio_connection import PigpioConnection


class HBridgeDriver:
    """Hardware abstraction for H-bridge motor driver via remote I2C.
    
    This class handles:
    - I2C communication through remote pigpio
    - Power and direction conversion
    - Hardware limits enforcement
    - Error handling and recovery
    """

    # PWM range constants
    PWM_MIN = 120  # Minimum PWM value that moves the motor
    PWM_MAX = 243  # Maximum PWM value
    
    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_address: int = 0x58,  # Default H-bridge address
        max_power: int = 100,
        mock_mode: bool = False,
        pigpio_host: Optional[str] = None,
        pigpio_port: Optional[int] = None,
        auto_stop_on_error: bool = True,
        error_threshold: int = 3,
    ):
        """Initialize the H-bridge driver.
        
        Args:
            i2c_bus: I2C bus number (typically 1 on Raspberry Pi)
            i2c_address: I2C address of the H-bridge controller (default 0x58)
            max_power: Maximum power percentage (0-100)
            mock_mode: If True, simulate I2C without actual hardware
            pigpio_host: IP address/hostname of remote Raspberry Pi (optional)
            pigpio_port: pigpiod port (optional, default 8888)
            auto_stop_on_error: If True, automatically stop motor on I2C errors (default True)
            error_threshold: Number of consecutive errors before triggering auto-stop (default 3)
        """
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.max_power = max_power
        self.mock_mode = mock_mode
        self.auto_stop_on_error = auto_stop_on_error
        self.error_threshold = error_threshold
        
        self._pigpio_conn = PigpioConnection()
        self._pi = None
        self._i2c_handle: Optional[int] = None
        self._connected = False
        self._last_direction = 0
        self._last_speed = 0
        self._error_count = 0
        self._consecutive_errors = 0
        
        self.logger = logging.getLogger('HBridgeDriver')
        
        # Configure pigpio connection
        if pigpio_host is not None or pigpio_port is not None or mock_mode:
            self._pigpio_conn.configure(
                host=pigpio_host,
                port=pigpio_port,
                mock_mode=mock_mode
            )
        
        # Try to initialize I2C
        self._initialize_i2c()
    
    def _initialize_i2c(self) -> bool:
        """Initialize I2C connection via remote pigpio.
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            self.logger.info("Running in MOCK mode - no actual I2C communication")
            self._connected = True
            return True
        
        try:
            # Get pigpio connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None:
                self.logger.error("Failed to get pigpio connection")
                self._connected = False
                return False
            
            # Close existing handle if present
            if self._i2c_handle is not None and self._i2c_handle >= 0:
                try:
                    self._pi.i2c_close(self._i2c_handle)
                except Exception:
                    pass  # Ignore errors on close
            
            # Open I2C bus
            self._i2c_handle = self._pi.i2c_open(self.i2c_bus, self.i2c_address)
            
            if self._i2c_handle < 0:
                self.logger.error(
                    f"Failed to open I2C bus {self.i2c_bus} at address 0x{self.i2c_address:02x}"
                )
                self._connected = False
                self._pigpio_conn.increment_error_count()
                return False
            
            self._connected = True
            self.logger.info(
                f"Remote I2C initialized: bus={self.i2c_bus}, address=0x{self.i2c_address:02x}, "
                f"handle={self._i2c_handle}, host={self._pigpio_conn.get_host()}"
            )
            return True
            
        except ImportError:
            self.logger.error(
                "pigpio library not installed. Install with: pip install pigpio"
            )
            self._connected = False
            return False
        except Exception as e:
            self.logger.error(f"Failed to initialize remote I2C: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return False
    
    def is_connected(self) -> bool:
        """Check if I2C connection is active.
        
        Returns:
            True if connected, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            return self._connected
        
        return self._connected and self._pigpio_conn.is_connected()
    
    def _power_to_pwm(self, power: int) -> int:
        """Convert power percentage to PWM value.
        
        Maps power (0-100) to PWM range with threshold:
        - power = 0        → PWM = 0
        - power = 1-100    → PWM = 120-243 (linear mapping)
        
        If the mapped PWM is between 0 and PWM_MIN (120), it's set to PWM_MIN
        to ensure the motor has enough power to move.
        
        Args:
            power: Absolute power value 0-100
        
        Returns:
            PWM value: 0 or 120-243
        """
        if power == 0:
            return 0
        
        # Linear mapping: power [1-100] -> PWM [120-243]
        # Formula: PWM = PWM_MIN + (power / 100) * (PWM_MAX - PWM_MIN)
        pwm = self.PWM_MIN + (power / 100.0) * (self.PWM_MAX - self.PWM_MIN)
        pwm = int(round(pwm))
        
        # Clamp to valid range
        pwm = max(0, min(self.PWM_MAX, pwm))
        
        # If below minimum threshold (but not 0), clamp to minimum
        # This ensures motor has enough power to actually move
        if self.PWM_MIN > pwm > 0:
            pwm = self.PWM_MIN
        
        return pwm
    
    def set_direction_speed(self, direction: int, speed: int) -> bool:
        """Set motor direction and speed.
        
        Args:
            direction: Motor direction
                       0 = stop
                       1 = forward (right for steering)
                       2 = reverse (left for steering)
            speed: Speed percentage 0-100
        
        Returns:
            True if command sent successfully, False otherwise
        """
        # Validate direction
        if direction not in [0, 1, 2]:
            self.logger.error(f"Invalid direction {direction}, must be 0, 1, or 2")
            return False
        
        # If direction is 0 (stop), always set speed to 0
        if direction == 0:
            speed = 0
        else:
            # Clamp speed to valid range
            speed = max(0, min(self.max_power, speed))
        
        # Send command to hardware
        success = self._send_i2c_command(direction, speed)
        
        if success:
            self._last_direction = direction
            self._last_speed = speed
            self._consecutive_errors = 0  # Reset on successful command
        else:
            self._error_count += 1
            self._consecutive_errors += 1
            
            # On error threshold, just log and reset counter
            # Don't close I2C connection as it affects other devices on the bus
            if self.auto_stop_on_error and self._consecutive_errors >= self.error_threshold:
                self.logger.warning(
                    f"I2C errors detected ({self._consecutive_errors}). "
                    "Motor likely hit steering limit. Will keep trying."
                )
                # Reset consecutive errors to allow new attempts
                self._consecutive_errors = 0
                # Store that we want motor stopped
                self._last_direction = 0
                self._last_speed = 0
        
        return success
    
    def _send_i2c_command(self, direction: int, speed: int) -> bool:
        """Send direction and speed command over I2C.
        
        Converts speed percentage to PWM value and sends to H-bridge.
        PWM mapping: 0 or 120-243 (values 1-119 are clamped to 120).
        
        Protocol (based on original ctrlSteer implementation):
        - I2C Address: 0x58
        - Register 0: Direction (0=stop, 1=right, 2=left for steering)
        - Register 2: PWM speed (0 or 120-243)
        
        Args:
            direction: 0=stop, 1=right, 2=left (for steering)
            speed: Speed percentage 0-100
        
        Returns:
            True if successful, False otherwise
        """
        # Convert speed percentage to PWM value
        # If direction is 0, PWM will be 0 regardless
        pwm = self._power_to_pwm(speed) if direction != 0 else 0
        
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            dir_name = {0: "STOP", 1: "LEFT", 2: "RIGHT"}.get(direction, "UNKNOWN")
            self.logger.debug(
                f"MOCK: Sending direction={direction} ({dir_name}), speed={speed}% → PWM={pwm}"
            )
            return True
        
        if not self._connected:
            # Don't log every time - too spammy
            if self._error_count % 10 == 0:
                self.logger.warning("I2C not connected, attempting to reconnect...")
            self._initialize_i2c()
            if not self._connected:
                return False
        
        try:
            # Ensure we have a valid connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None:
                self.logger.error("Lost pigpio connection")
                self._connected = False
                self._pigpio_conn.increment_error_count()
                return False
            
            # Verify handle is still valid
            if self._i2c_handle is None or self._i2c_handle < 0:
                self.logger.error("Invalid I2C handle, attempting to reconnect...")
                self._initialize_i2c()
                if not self._connected:
                    return False
            
            # Send to H-bridge via I2C (original protocol)
            # Register 0: Direction (0=stop, 1=right, 2=left)
            # Register 2: PWM speed value (0 or 120-243)
            self._pi.i2c_write_byte_data(self._i2c_handle, 0, direction)
            self._pi.i2c_write_byte_data(self._i2c_handle, 2, pwm)
            
            dir_name = {0: "STOP", 1: "LEFT", 2: "RIGHT"}.get(direction, "?")
            self.logger.debug(
                f"I2C sent: dir={direction}({dir_name}), speed={speed}% → PWM={pwm}"
            )
            return True
            
        except Exception as e:
            self.logger.error(f"I2C communication error: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return False
    
    def get_consecutive_errors(self) -> int:
        """Get the number of consecutive I2C errors.
        
        Returns:
            Consecutive error count
        """
        return self._consecutive_errors
    
    def stop(self) -> bool:
        """Emergency stop - set direction to 0 and speed to 0.
        
        Returns:
            True if successful, False otherwise
        """
        return self.set_direction_speed(0, 0)
    
    def get_last_direction(self) -> int:
        """Get the last commanded direction.
        
        Returns:
            Last direction (0, 1, or 2)
        """
        return self._last_direction
    
    def get_last_speed(self) -> int:
        """Get the last commanded speed.
        
        Returns:
            Last speed percentage (0-100)
        """
        return self._last_speed
    
    def get_error_count(self) -> int:
        """Get the number of I2C errors encountered.
        
        Returns:
            Error count
        """
        return self._error_count
    
    def reset_error_count(self) -> None:
        """Reset the error counter."""
        self._error_count = 0
        self._consecutive_errors = 0
    
    def close(self) -> None:
        """Close I2C connection and cleanup.
        
        Sends a final stop command (direction=0, speed=0) to turn off the H-bridge.
        Note: This doesn't disconnect the shared pigpio connection, as other
        drivers may still be using it.
        """
        try:
            # Send final stop command to turn off H-bridge
            self.logger.info("Sending final stop command to H-bridge...")
            self.set_direction_speed(0, 0)
        except Exception as e:
            self.logger.error(f"Error sending final stop command: {e}")
        
        if not self.mock_mode and not self._pigpio_conn.is_mock_mode():
            if self._pi is not None and self._i2c_handle is not None and self._i2c_handle >= 0:
                try:
                    self._pi.i2c_close(self._i2c_handle)
                    self.logger.info("I2C handle closed")
                except Exception as e:
                    self.logger.error(f"Error closing I2C handle: {e}")
        
        self._connected = False
        self._i2c_handle = None
        self._pi = None

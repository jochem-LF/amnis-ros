"""PWM driver for powertrain throttle control via remote pigpio.

This module provides a hardware abstraction layer for controlling the vehicle's
throttle via PWM (Pulse Width Modulation) on a GPIO pin. It handles low-level
GPIO communication through a remote Raspberry Pi running pigpio daemon.

Compatible with any system that can connect to a pigpiod daemon over network.
"""

from typing import Optional
import logging
from . import pigpio_connection


class PWMDriver:
    """Hardware abstraction for throttle control via PWM on remote GPIO.
    
    This class handles:
    - Remote GPIO initialization and cleanup via pigpio
    - PWM signal generation for throttle control
    - Safety limits and error handling
    - Mock mode for testing without hardware
    
    PWM Configuration:
    - Pin: Configurable (default BCM GPIO 22 = physical pin 15)
    - Frequency: 1kHz (typical for motor controllers)
    - Duty cycle: 0-100% (maps to throttle 0.0-1.0)
    
    Note: pigpio uses BCM (Broadcom) pin numbering, not physical pin numbers.
    """

    # PWM configuration
    DEFAULT_PWM_PIN = 22  # BCM GPIO 22 (physical pin 15 on Raspberry Pi)
    DEFAULT_PWM_FREQUENCY = 1000  # 1kHz
    
    def __init__(
        self,
        pwm_pin: int = DEFAULT_PWM_PIN,
        pwm_frequency: int = DEFAULT_PWM_FREQUENCY,
        max_throttle: float = 1.0,
        mock_mode: bool = False,
    ):
        """Initialize the PWM driver.
        
        Args:
            pwm_pin: GPIO pin number for PWM output (BCM numbering, default 22 = physical pin 15)
            pwm_frequency: PWM frequency in Hz (default 1kHz)
            max_throttle: Maximum throttle value 0.0-1.0 (default 1.0 = 100%)
            mock_mode: If True, simulate GPIO without actual hardware
            
        Note:
            Connection to pigpiod is configured via environment variables:
            - PIGPIO_HOST: Remote Raspberry Pi address (default: localhost)
            - PIGPIO_PORT: pigpiod port (default: 8888)
            - PIGPIO_MOCK_MODE: Set to 'true' for mock mode (default: false)
        """
        self.pwm_pin = pwm_pin
        self.pwm_frequency = pwm_frequency
        self.max_throttle = max(0.0, min(1.0, max_throttle))  # Clamp to [0, 1]
        self.mock_mode = mock_mode
        
        self._pi = None
        self._connected = False
        self._last_throttle = 0.0
        self._error_count = 0
        
        # Get connection config from environment
        self._host, self._port, env_mock = pigpio_connection.get_config()
        # Local mock_mode overrides environment
        if mock_mode:
            env_mock = True
        self.mock_mode = env_mock
        
        self.logger = logging.getLogger('PWMDriver')
        
        # Try to initialize GPIO
        self._initialize_gpio()
    
    def _initialize_gpio(self) -> bool:
        """Initialize GPIO and PWM via remote pigpio.
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode:
            self.logger.info("Running in MOCK mode - no actual GPIO communication")
            self._connected = True
            return True
        
        try:
            # Import pigpio and connect directly
            import pigpio
            
            self.logger.info(f"Connecting to pigpiod at {self._host}:{self._port}...")
            self._pi = pigpio.pi(self._host, self._port)
            
            if not self._pi.connected:
                self.logger.error(
                    f"Failed to connect to pigpiod at {self._host}:{self._port}. "
                    "Make sure pigpiod is running: sudo pigpiod"
                )
                self._connected = False
                return False
            
            # Set pin as output
            self._pi.set_mode(self.pwm_pin, pigpio.OUTPUT)
            
            # Set PWM frequency
            actual_freq = self._pi.set_PWM_frequency(self.pwm_pin, self.pwm_frequency)
            if actual_freq != self.pwm_frequency:
                self.logger.warning(
                    f"Requested frequency {self.pwm_frequency}Hz, got {actual_freq}Hz"
                )
            
            # Start with 0% duty cycle (no throttle)
            self._pi.set_PWM_dutycycle(self.pwm_pin, 0)
            
            self._connected = True
            self.logger.info(
                f"Remote GPIO PWM initialized: BCM GPIO {self.pwm_pin}, "
                f"frequency={actual_freq}Hz, host={self._host}:{self._port}"
            )
            return True
            
        except ImportError:
            self.logger.error(
                "pigpio library not installed. Install with: pip install pigpio"
            )
            self._connected = False
            return False
        except Exception as e:
            self.logger.error(f"Failed to initialize remote GPIO: {e}")
            self._connected = False
            return False
    
    def is_connected(self) -> bool:
        """Check if GPIO connection is active.
        
        Returns:
            True if connected, False otherwise
        """
        if self.mock_mode:
            return self._connected
        
        return self._connected and self._pi is not None and self._pi.connected
    
    def set_throttle(self, throttle: float) -> bool:
        """Set throttle value.
        
        Args:
            throttle: Throttle value in range [0.0, 1.0]
                     where 0.0 = no throttle, 1.0 = full throttle
        
        Returns:
            True if command accepted, False otherwise
        """
        # Validate throttle range
        if not (0.0 <= throttle <= 1.0):
            self.logger.error(f"Invalid throttle {throttle}, must be in [0.0, 1.0]")
            return False
        
        # Apply max throttle limit
        throttle = min(throttle, self.max_throttle)
        
        # Convert to duty cycle (0-100%)
        duty_cycle = throttle * 100.0
        
        # Send to hardware
        success = self._set_pwm_duty_cycle(duty_cycle)
        
        if success:
            self._last_throttle = throttle
        else:
            self._error_count += 1
        
        return success
    
    def _set_pwm_duty_cycle(self, duty_cycle: float) -> bool:
        """Set PWM duty cycle.
        
        Args:
            duty_cycle: Duty cycle percentage 0.0-100.0
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode:
            self.logger.debug(f"MOCK: Setting PWM duty cycle to {duty_cycle:.1f}%")
            return True
        
        if not self._connected:
            self.logger.error("GPIO not connected. Please restart the node.")
            return False
        
        try:
            # Check if connection is still valid
            if self._pi is None or not self._pi.connected:
                self.logger.error("Lost pigpio connection. Please restart the node.")
                self._connected = False
                return False
            
            # Convert percentage (0-100) to pigpio duty cycle (0-255)
            pigpio_duty = int((duty_cycle / 100.0) * 255.0)
            pigpio_duty = max(0, min(255, pigpio_duty))  # Clamp to valid range
            
            self._pi.set_PWM_dutycycle(self.pwm_pin, pigpio_duty)
            self.logger.debug(f"PWM duty cycle set to {duty_cycle:.1f}% (pigpio: {pigpio_duty}/255)")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set PWM duty cycle: {e}")
            self._connected = False
            return False
    
    def stop(self) -> bool:
        """Emergency stop - set throttle to 0.
        
        Returns:
            True if successful, False otherwise
        """
        return self.set_throttle(0.0)
    
    def get_last_throttle(self) -> float:
        """Get the last commanded throttle.
        
        Returns:
            Last throttle value [0.0, 1.0]
        """
        return self._last_throttle
    
    def get_error_count(self) -> int:
        """Get the number of GPIO errors encountered.
        
        Returns:
            Error count
        """
        return self._error_count
    
    def reset_error_count(self) -> None:
        """Reset the error counter."""
        self._error_count = 0
    
    def close(self) -> None:
        """Close GPIO connection and cleanup.
        
        Sends a final zero throttle command and cleans up GPIO resources.
        Note: This doesn't disconnect the shared pigpio connection, as other
        drivers may still be using it.
        """
        try:
            self.logger.info("Stopping PWM driver...")
            
            # Send final stop command
            self.set_throttle(0.0)
            
            # Set pin to low state and disconnect
            if not self.mock_mode and self._pi is not None:
                try:
                    self._pi.set_PWM_dutycycle(self.pwm_pin, 0)
                    self._pi.stop()
                    self.logger.info("PWM stopped and disconnected")
                except Exception as e:
                    self.logger.error(f"Error stopping PWM: {e}")
                    
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
        
        self._connected = False
        self._pi = None

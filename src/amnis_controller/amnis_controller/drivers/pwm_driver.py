"""PWM driver for powertrain throttle control via GPIO.

This module provides a hardware abstraction layer for controlling the vehicle's
throttle via PWM (Pulse Width Modulation) on a GPIO pin. It handles low-level
GPIO communication, PWM signal generation, and error handling.

Compatible with Jetson Orin and other Linux systems with GPIO support.
"""

from typing import Optional
import logging


class PWMDriver:
    """Hardware abstraction for throttle control via PWM on GPIO.
    
    This class handles:
    - GPIO initialization and cleanup
    - PWM signal generation for throttle control
    - Safety limits and error handling
    - Mock mode for testing without hardware
    
    PWM Configuration:
    - Pin: Configurable (default pin 15)
    - Frequency: 1kHz (typical for motor controllers)
    - Duty cycle: 0-100% (maps to throttle 0.0-1.0)
    """

    # PWM configuration
    DEFAULT_PWM_PIN = 15  # Physical pin number
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
            pwm_pin: GPIO pin number for PWM output (physical pin numbering)
            pwm_frequency: PWM frequency in Hz (default 1kHz)
            max_throttle: Maximum throttle value 0.0-1.0 (default 1.0 = 100%)
            mock_mode: If True, simulate GPIO without actual hardware
        """
        self.pwm_pin = pwm_pin
        self.pwm_frequency = pwm_frequency
        self.max_throttle = max(0.0, min(1.0, max_throttle))  # Clamp to [0, 1]
        self.mock_mode = mock_mode
        
        self._gpio = None
        self._pwm: Optional[object] = None
        self._connected = False
        self._last_throttle = 0.0
        self._error_count = 0
        
        self.logger = logging.getLogger('PWMDriver')
        
        # Try to initialize GPIO
        self._initialize_gpio()
    
    def _initialize_gpio(self) -> bool:
        """Initialize GPIO and PWM.
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode:
            self.logger.info("Running in MOCK mode - no actual GPIO communication")
            self._connected = True
            return True
        
        try:
            # Try to import Jetson.GPIO
            import Jetson.GPIO as GPIO
            self._gpio = GPIO
            
            # Set pin numbering mode to BOARD (physical pin numbers)
            self._gpio.setmode(self._gpio.BOARD)
            
            # Setup PWM pin
            self._gpio.setup(self.pwm_pin, self._gpio.OUT)
            
            # Create PWM object
            self._pwm = self._gpio.PWM(self.pwm_pin, self.pwm_frequency)
            
            # Start PWM with 0% duty cycle (no throttle)
            self._pwm.start(0)
            
            self._connected = True
            self.logger.info(
                f"GPIO PWM initialized: pin={self.pwm_pin}, "
                f"frequency={self.pwm_frequency}Hz"
            )
            return True
            
        except ImportError:
            self.logger.error(
                "Jetson.GPIO not installed. Install with: pip install Jetson.GPIO"
            )
            self._connected = False
            return False
        except Exception as e:
            self.logger.error(f"Failed to initialize GPIO: {e}")
            self._connected = False
            return False
    
    def is_connected(self) -> bool:
        """Check if GPIO connection is active.
        
        Returns:
            True if connected, False otherwise
        """
        return self._connected
    
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
        
        if not self._connected or self._pwm is None:
            self.logger.warning("GPIO not connected, attempting to reconnect...")
            self._initialize_gpio()
            if not self._connected:
                return False
        
        try:
            self._pwm.ChangeDutyCycle(duty_cycle)
            self.logger.debug(f"PWM duty cycle set to {duty_cycle:.1f}%")
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
        """
        try:
            self.logger.info("Stopping PWM driver...")
            
            # Send final stop command
            self.set_throttle(0.0)
            
            # Stop PWM
            if self._pwm is not None and not self.mock_mode:
                try:
                    self._pwm.stop()
                    self.logger.info("PWM stopped")
                except Exception as e:
                    self.logger.error(f"Error stopping PWM: {e}")
            
            # Cleanup GPIO
            if self._gpio is not None and not self.mock_mode:
                try:
                    self._gpio.cleanup()
                    self.logger.info("GPIO cleaned up")
                except Exception as e:
                    self.logger.error(f"Error cleaning up GPIO: {e}")
                    
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
        
        self._connected = False


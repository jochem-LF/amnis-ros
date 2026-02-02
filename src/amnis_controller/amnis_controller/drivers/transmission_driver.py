"""Transmission relay driver for gear control via remote pigpio GPIO.

This module provides a hardware abstraction layer for controlling the vehicle's
transmission system via three GPIO relays through a remote Raspberry Pi running
pigpio daemon. It handles gear state management and safety features.

Transmission Logic:
- Gear 0 (neutral): disable_neutral=LOW, enable_reverse=LOW
- Gear 1 (forward): disable_neutral=HIGH, enable_reverse=LOW
- Gear -1 (reverse): disable_neutral=HIGH, enable_reverse=HIGH
- External mode: Controlled independently by vehicle state
"""

from typing import Optional
import logging
from .pigpio_connection import PigpioConnection


class TransmissionDriver:
    """Hardware abstraction for transmission control via GPIO relays.
    
    This class handles:
    - Remote GPIO initialization and cleanup via pigpio
    - Transmission gear control (neutral, forward, reverse)
    - External mode relay control
    - Safety features and error handling
    - Mock mode for testing without hardware
    
    Relay Configuration:
    - disable_neutral_pin: Controls neutral lock (LOW=neutral, HIGH=not neutral)
    - enable_reverse_pin: Controls reverse enable (LOW=forward, HIGH=reverse)
    - external_mode_pin: Controls external mode (LOW=off, HIGH=on)
    
    Note: pigpio uses BCM (Broadcom) pin numbering.
    """

    # Default GPIO pins (BCM numbering)
    DEFAULT_DISABLE_NEUTRAL_PIN = 17  # BCM GPIO 17 (physical pin 11)
    DEFAULT_ENABLE_REVERSE_PIN = 27   # BCM GPIO 27 (physical pin 13)
    DEFAULT_EXTERNAL_MODE_PIN = 23    # BCM GPIO 23 (physical pin 16)
    
    # Gear constants
    GEAR_REVERSE = -1
    GEAR_NEUTRAL = 0
    GEAR_FORWARD = 1
    
    def __init__(
        self,
        disable_neutral_pin: int = DEFAULT_DISABLE_NEUTRAL_PIN,
        enable_reverse_pin: int = DEFAULT_ENABLE_REVERSE_PIN,
        external_mode_pin: Optional[int] = DEFAULT_EXTERNAL_MODE_PIN,
        mock_mode: bool = False,
        pigpio_host: Optional[str] = None,
        pigpio_port: Optional[int] = None,
    ):
        """Initialize the transmission driver.
        
        Args:
            disable_neutral_pin: GPIO pin for neutral disable relay (BCM numbering)
            enable_reverse_pin: GPIO pin for reverse enable relay (BCM numbering)
            external_mode_pin: GPIO pin for external mode relay (BCM numbering, None to skip)
            mock_mode: If True, simulate GPIO without actual hardware
            pigpio_host: IP address/hostname of remote Raspberry Pi (optional)
            pigpio_port: pigpiod port (optional, default 8888)
        """
        self.disable_neutral_pin = disable_neutral_pin
        self.enable_reverse_pin = enable_reverse_pin
        self.external_mode_pin = external_mode_pin
        self.use_external_mode = external_mode_pin is not None
        self.mock_mode = mock_mode
        
        self._pigpio_conn = PigpioConnection()
        self._pi = None
        self._connected = False
        self._current_gear = self.GEAR_NEUTRAL
        self._external_mode = False
        self._error_count = 0
        
        self.logger = logging.getLogger('TransmissionDriver')
        
        # Configure pigpio connection
        if pigpio_host is not None or pigpio_port is not None or mock_mode:
            self._pigpio_conn.configure(
                host=pigpio_host,
                port=pigpio_port,
                mock_mode=mock_mode
            )
        
        # Try to initialize GPIO
        self._initialize_gpio()
    
    def _initialize_gpio(self) -> bool:
        """Initialize GPIO pins for relay control via remote pigpio.
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            self.logger.info("Running in MOCK mode - no actual GPIO communication")
            self._connected = True
            return True
        
        try:
            # Get pigpio connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None:
                self.logger.error("Failed to get pigpio connection")
                self._connected = False
                return False
            
            # Import pigpio for constants
            import pigpio
            
            # Set pins as outputs
            self._pi.set_mode(self.disable_neutral_pin, pigpio.OUTPUT)
            self._pi.set_mode(self.enable_reverse_pin, pigpio.OUTPUT)
            if self.use_external_mode:
                self._pi.set_mode(self.external_mode_pin, pigpio.OUTPUT)
            
            # Set initial safe state directly without using _set_gpio_state (to avoid recursion)
            self._pi.write(self.disable_neutral_pin, 0)  # LOW = neutral
            self._pi.write(self.enable_reverse_pin, 0)   # LOW = not reverse
            if self.use_external_mode:
                self._pi.write(self.external_mode_pin, 0)    # LOW = external mode off
            
            self._connected = True
            self._current_gear = self.GEAR_NEUTRAL
            self._external_mode = False
            
            ext_mode_str = f", external_mode={self.external_mode_pin}" if self.use_external_mode else ""
            self.logger.info(
                f"Remote GPIO transmission initialized: "
                f"disable_neutral={self.disable_neutral_pin}, "
                f"enable_reverse={self.enable_reverse_pin}{ext_mode_str} (BCM), "
                f"host={self._pigpio_conn.get_host()}"
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
            self._pigpio_conn.increment_error_count()
            return False
    
    def is_connected(self) -> bool:
        """Check if GPIO connection is active.
        
        Returns:
            True if connected, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            return self._connected
        
        return self._connected and self._pigpio_conn.is_connected()
    
    def _set_gpio_state(
        self,
        disable_neutral: bool,
        enable_reverse: bool,
        external_mode: Optional[bool] = None
    ) -> bool:
        """Set the state of the GPIO relays.
        
        Args:
            disable_neutral: True=HIGH (not neutral), False=LOW (neutral)
            enable_reverse: True=HIGH (reverse), False=LOW (forward)
            external_mode: Optional. If provided, sets external mode state
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            ext_str = f", external_mode={external_mode}" if external_mode is not None else ""
            self.logger.debug(
                f"MOCK: Setting relays: disable_neutral={disable_neutral}, "
                f"enable_reverse={enable_reverse}{ext_str}"
            )
            return True
        
        if not self._connected:
            self.logger.error("GPIO not connected, cannot set relay state")
            return False
        
        try:
            # Ensure we have a valid connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None:
                self.logger.error("Lost pigpio connection")
                self._connected = False
                self._pigpio_conn.increment_error_count()
                return False
            
            # Set relay states (True=HIGH=1, False=LOW=0)
            self._pi.write(self.disable_neutral_pin, 1 if disable_neutral else 0)
            self._pi.write(self.enable_reverse_pin, 1 if enable_reverse else 0)
            
            # Optionally set external mode
            if external_mode is not None:
                self._pi.write(self.external_mode_pin, 1 if external_mode else 0)
                self._external_mode = external_mode
            
            self.logger.debug(
                f"Relays set: disable_neutral={disable_neutral}, "
                f"enable_reverse={enable_reverse}"
            )
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set GPIO state: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return False
    
    def set_gear(self, gear: int) -> bool:
        """Set transmission gear.
        
        Args:
            gear: Gear position
                  -1 = reverse
                   0 = neutral
                   1 = forward
        
        Returns:
            True if command accepted, False otherwise
        """
        # Validate gear range
        if gear not in [self.GEAR_REVERSE, self.GEAR_NEUTRAL, self.GEAR_FORWARD]:
            self.logger.error(
                f"Invalid gear {gear}, must be -1 (reverse), 0 (neutral), or 1 (forward)"
            )
            return False
        
        # Determine relay states based on gear
        if gear == self.GEAR_NEUTRAL:
            # Neutral: disable_neutral=LOW, enable_reverse=LOW
            disable_neutral = False
            enable_reverse = False
            gear_name = "NEUTRAL"
        elif gear == self.GEAR_FORWARD:
            # Forward: disable_neutral=HIGH, enable_reverse=LOW
            disable_neutral = True
            enable_reverse = False
            gear_name = "FORWARD"
        elif gear == self.GEAR_REVERSE:
            # Reverse: disable_neutral=HIGH, enable_reverse=HIGH
            disable_neutral = True
            enable_reverse = True
            gear_name = "REVERSE"
        
        # Send to hardware
        success = self._set_gpio_state(disable_neutral, enable_reverse)
        
        if success:
            if self._current_gear != gear:
                self.logger.info(f"Gear changed: {self._current_gear} → {gear} ({gear_name})")
            self._current_gear = gear
        else:
            self._error_count += 1
        
        return success
    
    def set_external_mode(self, enabled: bool) -> bool:
        """Set external mode relay state.
        
        Args:
            enabled: True to enable external mode (HIGH), False to disable (LOW)
        
        Returns:
            True if successful, False otherwise
        """
        if not self.use_external_mode:
            # This instance doesn't control external mode
            return True
        
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            self.logger.debug(f"MOCK: Setting external mode to {enabled}")
            self._external_mode = enabled
            return True
        
        if not self._connected:
            self.logger.error("GPIO not connected, cannot set external mode")
            return False
        
        try:
            # Ensure we have a valid connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None:
                self.logger.error("Lost pigpio connection")
                self._connected = False
                self._pigpio_conn.increment_error_count()
                return False
            
            # Set external mode relay
            self._pi.write(self.external_mode_pin, 1 if enabled else 0)
            
            if self._external_mode != enabled:
                self.logger.info(f"External mode: {self._external_mode} → {enabled}")
            self._external_mode = enabled
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to set external mode: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return False
    
    def emergency_neutral(self) -> bool:
        """Emergency neutral - force transmission to neutral state.
        
        Returns:
            True if successful, False otherwise
        """
        self.logger.warning("Emergency neutral activated!")
        return self.set_gear(self.GEAR_NEUTRAL)
    
    def get_current_gear(self) -> int:
        """Get the current gear state.
        
        Returns:
            Current gear (-1, 0, or 1)
        """
        return self._current_gear
    
    def is_external_mode(self) -> bool:
        """Check if external mode is enabled.
        
        Returns:
            True if external mode is enabled, False otherwise
        """
        return self._external_mode
    
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
        
        Forces neutral state and external mode off before cleanup.
        Note: This doesn't disconnect the shared pigpio connection, as other
        drivers may still be using it.
        """
        try:
            self.logger.info("Stopping transmission driver...")
            
            # Force safe state: neutral and external mode off
            self.emergency_neutral()
            self.set_external_mode(False)
            
            # Set all pins to LOW
            if not self.mock_mode and not self._pigpio_conn.is_mock_mode():
                if self._pi is not None:
                    try:
                        self._pi.write(self.disable_neutral_pin, 0)
                        self._pi.write(self.enable_reverse_pin, 0)
                        if self.use_external_mode:
                            self._pi.write(self.external_mode_pin, 0)
                        self.logger.info("All transmission relays set to LOW")
                    except Exception as e:
                        self.logger.error(f"Error setting pins to LOW: {e}")
                    
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
        
        self._connected = False
        self._pi = None

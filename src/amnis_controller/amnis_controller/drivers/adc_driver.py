"""ADS1015L ADC driver for reading potentiometers via I2C using remote pigpio.

This module provides a hardware abstraction layer for reading analog values from
an ADS1015L 12-bit ADC chip connected via I2C. It handles reading gas pedal and
steering wheel potentiometers with dynamic calibration and normalization support.

Hardware Configuration:
- ADS1015L 12-bit ADC at I2C address 0x48 (default)
- Gas pedal potentiometer on AIN0
- Steering wheel potentiometer on AIN1
- PGA setting: ±4.096V range
- Conversion rate: 1600 samples per second
"""

from typing import Optional, Tuple
import logging
import time
from .pigpio_connection import PigpioConnection


class ADCDriver:
    """Hardware abstraction for ADS1015L ADC via I2C.
    
    This class handles:
    - Remote I2C communication via pigpio
    - 12-bit ADC reading from ADS1015L
    - Dynamic calibration for potentiometer ranges
    - Normalized output (0.0 to 1.0)
    - Mock mode for testing without hardware
    
    ADC Configuration:
    - Resolution: 12-bit (0-2047 for positive values)
    - Channels: AIN0 (gas pedal), AIN1 (steering wheel)
    - Voltage range: 0-4.096V (with PGA=±4.096V)
    - Mode: Single-shot conversion
    
    Note: pigpio uses BCM pin numbering for I2C bus selection.
    """

    # I2C configuration
    DEFAULT_I2C_BUS = 1  # I2C bus 1 on Raspberry Pi
    DEFAULT_I2C_ADDRESS = 0x48  # ADS1015L default address (ADDR to GND)
    
    # ADS1015L register addresses
    REG_CONVERSION = 0x00
    REG_CONFIG = 0x01
    
    # Config register bits
    OS_SINGLE = 0x8000       # Start single conversion
    MUX_AIN0 = 0x4000        # Single-ended AIN0
    MUX_AIN1 = 0x5000        # Single-ended AIN1
    MUX_AIN2 = 0x6000        # Single-ended AIN2
    MUX_AIN3 = 0x7000        # Single-ended AIN3
    PGA_4V = 0x0200          # +/- 4.096V range
    MODE_SINGLE = 0x0100     # Single-shot mode
    DR_1600SPS = 0x0080      # 1600 samples per second
    COMP_QUE_DISABLE = 0x0003  # Disable comparator
    
    # Base config (without MUX setting)
    CONFIG_BASE = OS_SINGLE | PGA_4V | MODE_SINGLE | DR_1600SPS | COMP_QUE_DISABLE
    
    # ADC parameters
    ADC_MAX_VALUE = 2047  # 12-bit max positive value
    ADC_VOLTAGE_RANGE = 4.096  # Volts
    
    # Channel mapping
    CHANNEL_GAS_PEDAL = 0  # AIN0
    CHANNEL_STEERING_WHEEL = 1  # AIN1
    
    def __init__(
        self,
        i2c_bus: int = DEFAULT_I2C_BUS,
        i2c_address: int = DEFAULT_I2C_ADDRESS,
        mock_mode: bool = False,
        pigpio_host: Optional[str] = None,
        pigpio_port: Optional[int] = None,
        # Calibration defaults (full range if not calibrated)
        gas_min: Optional[int] = None,
        gas_max: Optional[int] = None,
        steer_min: Optional[int] = None,
        steer_max: Optional[int] = None,
    ):
        """Initialize the ADC driver.
        
        Args:
            i2c_bus: I2C bus number (1 for Raspberry Pi)
            i2c_address: I2C address of ADS1015L (default 0x48)
            mock_mode: If True, simulate I2C without actual hardware
            pigpio_host: IP address/hostname of remote Raspberry Pi (optional)
            pigpio_port: pigpiod port (optional, default 8888)
            gas_min: Minimum raw ADC value for gas pedal calibration (optional)
            gas_max: Maximum raw ADC value for gas pedal calibration (optional)
            steer_min: Minimum raw ADC value for steering wheel calibration (optional)
            steer_max: Maximum raw ADC value for steering wheel calibration (optional)
        """
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.mock_mode = mock_mode
        
        self._pigpio_conn = PigpioConnection()
        self._pi = None
        self._i2c_handle = None
        self._connected = False
        self._error_count = 0
        
        # Calibration data - use provided values or defaults
        self._gas_min = gas_min if gas_min is not None else 0
        self._gas_max = gas_max if gas_max is not None else self.ADC_MAX_VALUE
        self._steer_min = steer_min if steer_min is not None else 0
        self._steer_max = steer_max if steer_max is not None else self.ADC_MAX_VALUE
        
        # Calibration state
        self._calibrating = False
        self._calib_gas_min = self.ADC_MAX_VALUE
        self._calib_gas_max = 0
        self._calib_steer_min = self.ADC_MAX_VALUE
        self._calib_steer_max = 0
        
        self.logger = logging.getLogger('ADCDriver')
        
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
        """Initialize I2C connection to ADS1015L via remote pigpio.
        
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
            
            # Open I2C device
            self._i2c_handle = self._pi.i2c_open(self.i2c_bus, self.i2c_address)
            
            if self._i2c_handle < 0:
                self.logger.error(
                    f"Failed to open I2C device at bus {self.i2c_bus}, "
                    f"address 0x{self.i2c_address:02X}"
                )
                self._connected = False
                return False
            
            self._connected = True
            self.logger.info(
                f"I2C ADC initialized: bus={self.i2c_bus}, "
                f"address=0x{self.i2c_address:02X}, "
                f"host={self._pigpio_conn.get_host()}"
            )
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize I2C: {e}")
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
    
    def _write_config(self, config: int) -> bool:
        """Write 16-bit config value to config register.
        
        Args:
            config: 16-bit configuration value
            
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            self.logger.debug(f"MOCK: Writing config 0x{config:04X}")
            return True
        
        if not self._connected:
            self.logger.error("I2C not connected, cannot write config")
            return False
        
        try:
            # Ensure we have a valid connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None or self._i2c_handle is None:
                self.logger.error("Lost pigpio connection")
                self._connected = False
                self._pigpio_conn.increment_error_count()
                return False
            
            # pigpio expects word data in host byte order, it handles the conversion
            # However, we need to swap bytes for ADS1015L (MSB first)
            msb = (config >> 8) & 0xFF
            lsb = config & 0xFF
            swapped = (lsb << 8) | msb
            
            self._pi.i2c_write_word_data(self._i2c_handle, self.REG_CONFIG, swapped)
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to write config: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return False
    
    def _read_conversion(self) -> Optional[int]:
        """Read 16-bit conversion result from conversion register.
        
        Returns:
            12-bit ADC value (0-2047) or None on error
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            # Return a mock value in the middle of the range
            import random
            return random.randint(500, 1500)
        
        if not self._connected:
            self.logger.error("I2C not connected, cannot read conversion")
            return None
        
        try:
            # Ensure we have a valid connection
            self._pi = self._pigpio_conn.get_pi()
            if self._pi is None or self._i2c_handle is None:
                self.logger.error("Lost pigpio connection")
                self._connected = False
                self._pigpio_conn.increment_error_count()
                return None
            
            # Read 16-bit word from conversion register
            raw = self._pi.i2c_read_word_data(self._i2c_handle, self.REG_CONVERSION)
            
            # Swap bytes (pigpio returns in host order, we need MSB first)
            msb = raw & 0xFF
            lsb = (raw >> 8) & 0xFF
            result = (msb << 8) | lsb
            
            # ADS1015L result is 12-bit, left-aligned in 16-bit register
            # Shift right by 4 to get 12-bit value
            value = result >> 4
            
            return value
            
        except Exception as e:
            self.logger.error(f"Failed to read conversion: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return None
    
    def read_raw(self, channel: int) -> Optional[int]:
        """Read raw ADC value from specified channel.
        
        Args:
            channel: ADC channel (0-3 for AIN0-AIN3)
            
        Returns:
            12-bit ADC value (0-2047) or None on error
        """
        if channel < 0 or channel > 3:
            self.logger.error(f"Invalid channel {channel}, must be 0-3")
            return None
        
        # Set MUX bits for single-ended input
        mux_values = [
            self.MUX_AIN0,
            self.MUX_AIN1,
            self.MUX_AIN2,
            self.MUX_AIN3
        ]
        config = self.CONFIG_BASE | mux_values[channel]
        
        # Start conversion
        if not self._write_config(config):
            return None
        
        # Wait for conversion to complete (~1ms at 1600 SPS)
        time.sleep(0.002)
        
        # Read result
        value = self._read_conversion()
        
        # Update calibration if in calibration mode
        if self._calibrating and value is not None:
            self._update_calibration_internal(channel, value)
        
        return value
    
    def read_voltage(self, channel: int) -> Optional[float]:
        """Read voltage from specified channel.
        
        Args:
            channel: ADC channel (0-3 for AIN0-AIN3)
            
        Returns:
            Voltage in volts (0.0-4.096V) or None on error
        """
        raw = self.read_raw(channel)
        if raw is None:
            return None
        
        # Calculate voltage: LSB = 4.096V / 2048 = 2mV
        voltage = raw * (self.ADC_VOLTAGE_RANGE / 2048.0)
        return voltage
    
    def _normalize_value(self, raw_value: int, min_val: int, max_val: int) -> float:
        """Normalize raw ADC value to 0.0-1.0 range.
        
        Args:
            raw_value: Raw ADC reading
            min_val: Calibrated minimum value
            max_val: Calibrated maximum value
            
        Returns:
            Normalized value (0.0 to 1.0)
        """
        # Handle edge case where min == max
        if max_val <= min_val:
            return 0.0
        
        # Normalize to 0.0-1.0
        normalized = (raw_value - min_val) / (max_val - min_val)
        
        # Clamp to valid range
        return max(0.0, min(1.0, normalized))
    
    def get_gas_pedal(self) -> Optional[float]:
        """Get normalized gas pedal value.
        
        Returns:
            Normalized value (0.0 to 1.0) or None on error
        """
        raw = self.read_raw(self.CHANNEL_GAS_PEDAL)
        if raw is None:
            return None
        
        return self._normalize_value(raw, self._gas_min, self._gas_max)
    
    def get_steering_wheel(self) -> Optional[float]:
        """Get normalized steering wheel value.
        
        Returns:
            Normalized value (0.0 to 1.0) or None on error
        """
        raw = self.read_raw(self.CHANNEL_STEERING_WHEEL)
        if raw is None:
            return None
        
        return self._normalize_value(raw, self._steer_min, self._steer_max)
    
    def start_calibration(self) -> None:
        """Start calibration mode.
        
        While in calibration mode, the driver will track min/max values
        for each channel. Call end_calibration() to save the values.
        """
        self._calibrating = True
        self._calib_gas_min = self.ADC_MAX_VALUE
        self._calib_gas_max = 0
        self._calib_steer_min = self.ADC_MAX_VALUE
        self._calib_steer_max = 0
        
        self.logger.info("Calibration started - move gas pedal and steering wheel through full range")
    
    def _update_calibration_internal(self, channel: int, value: int) -> None:
        """Internal method to update calibration values during calibration mode.
        
        Args:
            channel: Channel number (0 or 1)
            value: Raw ADC value
        """
        if channel == self.CHANNEL_GAS_PEDAL:
            self._calib_gas_min = min(self._calib_gas_min, value)
            self._calib_gas_max = max(self._calib_gas_max, value)
        elif channel == self.CHANNEL_STEERING_WHEEL:
            self._calib_steer_min = min(self._calib_steer_min, value)
            self._calib_steer_max = max(self._calib_steer_max, value)
    
    def update_calibration(self) -> None:
        """Explicitly update calibration by reading current values.
        
        This is useful if you want to manually trigger readings during calibration
        instead of relying on automatic updates during read_raw() calls.
        """
        if not self._calibrating:
            self.logger.warning("Not in calibration mode")
            return
        
        # Read both channels to update calibration
        self.read_raw(self.CHANNEL_GAS_PEDAL)
        self.read_raw(self.CHANNEL_STEERING_WHEEL)
    
    def end_calibration(self) -> bool:
        """End calibration mode and save calibration values.
        
        Returns:
            True if calibration was successful, False if insufficient data
        """
        if not self._calibrating:
            self.logger.warning("Not in calibration mode")
            return False
        
        self._calibrating = False
        
        # Validate calibration data
        gas_valid = self._calib_gas_max > self._calib_gas_min
        steer_valid = self._calib_steer_max > self._calib_steer_min
        
        if not gas_valid or not steer_valid:
            self.logger.error(
                "Calibration failed - insufficient range detected. "
                f"Gas: {self._calib_gas_min}-{self._calib_gas_max}, "
                f"Steer: {self._calib_steer_min}-{self._calib_steer_max}"
            )
            return False
        
        # Save calibration values
        self._gas_min = self._calib_gas_min
        self._gas_max = self._calib_gas_max
        self._steer_min = self._calib_steer_min
        self._steer_max = self._calib_steer_max
        
        self.logger.info(
            f"Calibration complete - "
            f"Gas: {self._gas_min}-{self._gas_max}, "
            f"Steer: {self._steer_min}-{self._steer_max}"
        )
        return True
    
    def reset_calibration(self) -> None:
        """Reset calibration to full ADC range."""
        self._gas_min = 0
        self._gas_max = self.ADC_MAX_VALUE
        self._steer_min = 0
        self._steer_max = self.ADC_MAX_VALUE
        
        self.logger.info("Calibration reset to full range")
    
    def load_calibration(
        self,
        gas_min: int,
        gas_max: int,
        steer_min: int,
        steer_max: int
    ) -> bool:
        """Load calibration values programmatically.
        
        Args:
            gas_min: Minimum raw ADC value for gas pedal
            gas_max: Maximum raw ADC value for gas pedal
            steer_min: Minimum raw ADC value for steering wheel
            steer_max: Maximum raw ADC value for steering wheel
            
        Returns:
            True if values are valid, False otherwise
        """
        # Validate ranges
        if gas_max <= gas_min or steer_max <= steer_min:
            self.logger.error("Invalid calibration values: max must be > min")
            return False
        
        if not (0 <= gas_min <= self.ADC_MAX_VALUE and 0 <= gas_max <= self.ADC_MAX_VALUE):
            self.logger.error(f"Gas calibration values out of range (0-{self.ADC_MAX_VALUE})")
            return False
        
        if not (0 <= steer_min <= self.ADC_MAX_VALUE and 0 <= steer_max <= self.ADC_MAX_VALUE):
            self.logger.error(f"Steering calibration values out of range (0-{self.ADC_MAX_VALUE})")
            return False
        
        self._gas_min = gas_min
        self._gas_max = gas_max
        self._steer_min = steer_min
        self._steer_max = steer_max
        
        self.logger.info(
            f"Calibration loaded - "
            f"Gas: {self._gas_min}-{self._gas_max}, "
            f"Steer: {self._steer_min}-{self._steer_max}"
        )
        return True
    
    def get_calibration(self) -> Tuple[int, int, int, int]:
        """Get current calibration values.
        
        Returns:
            Tuple of (gas_min, gas_max, steer_min, steer_max)
        """
        return (self._gas_min, self._gas_max, self._steer_min, self._steer_max)
    
    def is_calibrating(self) -> bool:
        """Check if currently in calibration mode.
        
        Returns:
            True if calibrating, False otherwise
        """
        return self._calibrating
    
    def get_error_count(self) -> int:
        """Get the number of I2C errors encountered.
        
        Returns:
            Error count
        """
        return self._error_count
    
    def reset_error_count(self) -> None:
        """Reset the error counter."""
        self._error_count = 0
    
    def close(self) -> None:
        """Close I2C connection and cleanup.
        
        Note: This doesn't disconnect the shared pigpio connection, as other
        drivers may still be using it.
        """
        try:
            self.logger.info("Closing ADC driver...")
            
            if not self.mock_mode and not self._pigpio_conn.is_mock_mode():
                if self._pi is not None and self._i2c_handle is not None:
                    try:
                        self._pi.i2c_close(self._i2c_handle)
                        self.logger.info("I2C device closed")
                    except Exception as e:
                        self.logger.error(f"Error closing I2C device: {e}")
                    
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")
        
        self._connected = False
        self._i2c_handle = None
        self._pi = None

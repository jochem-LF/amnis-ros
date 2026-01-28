"""ADS1015L ADC driver for analog sensor input via remote pigpio I2C.

This module provides a hardware abstraction layer for reading analog sensors
through an ADS1015L 12-bit ADC over I2C. It communicates with a remote 
Raspberry Pi running pigpio daemon.

The ADS1015L is a 4-channel, 12-bit ADC with I2C interface, commonly used
for reading potentiometers, analog sensors, and other voltage inputs.

Compatible with any system that can connect to a pigpiod daemon over network.
"""

from typing import Optional
import logging
import time
from .pigpio_connection import PigpioConnection


class ADCDriver:
    """Hardware abstraction for ADS1015L ADC via remote I2C.
    
    This class handles:
    - I2C communication through remote pigpio
    - 12-bit ADC conversions from ADS1015L
    - Voltage conversion with configurable PGA
    - Channel multiplexing (AIN0-AIN3)
    - Error handling and recovery
    
    ADC Configuration:
    - Channels: 4 single-ended inputs (AIN0-AIN3)
    - Resolution: 12-bit (0-2047 for positive values)
    - PGA: ±4.096V range (suitable for 0-3.3V sensors)
    - Sample rate: 1600 SPS
    - Mode: Single-shot conversion (power efficient)
    """

    # I2C Register addresses
    REG_CONVERSION = 0x00  # Conversion result register
    REG_CONFIG = 0x01      # Configuration register
    
    # Configuration register bits
    OS_SINGLE = 0x8000        # Start single conversion
    MUX_AIN0 = 0x4000         # Single-ended AIN0
    MUX_AIN1 = 0x5000         # Single-ended AIN1
    MUX_AIN2 = 0x6000         # Single-ended AIN2
    MUX_AIN3 = 0x7000         # Single-ended AIN3
    PGA_4V = 0x0200           # ±4.096V range (good for 3.3V input)
    MODE_SINGLE = 0x0100      # Single-shot mode
    DR_1600SPS = 0x0080       # 1600 samples per second
    COMP_QUE_DISABLE = 0x0003 # Disable comparator
    
    # Base config (without MUX setting)
    CONFIG_BASE = OS_SINGLE | PGA_4V | MODE_SINGLE | DR_1600SPS | COMP_QUE_DISABLE
    
    # MUX values for each channel
    MUX_VALUES = {
        0: MUX_AIN0,
        1: MUX_AIN1,
        2: MUX_AIN2,
        3: MUX_AIN3,
    }
    
    # Voltage conversion constant for PGA_4V
    # 12-bit ADC: 2048 steps for positive range
    # ±4.096V range means 4.096V / 2048 = 2mV per LSB
    VOLTS_PER_LSB = 4.096 / 2048.0
    
    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_address: int = 0x48,
        mock_mode: bool = False,
        pigpio_host: Optional[str] = None,
        pigpio_port: Optional[int] = None,
    ):
        """Initialize the ADC driver.
        
        Args:
            i2c_bus: I2C bus number (typically 1 on Raspberry Pi)
            i2c_address: I2C address of the ADS1015L
                        0x48 = ADDR to GND (default)
                        0x49 = ADDR to VDD
                        0x4A = ADDR to SDA
                        0x4B = ADDR to SCL
            mock_mode: If True, simulate I2C without actual hardware
            pigpio_host: IP address/hostname of remote Raspberry Pi (optional)
            pigpio_port: pigpiod port (optional, default 8888)
        """
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.mock_mode = mock_mode
        
        self._pigpio_conn = PigpioConnection()
        self._pi = None
        self._i2c_handle: Optional[int] = None
        self._connected = False
        self._error_count = 0
        
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
                f"Remote I2C ADC initialized: bus={self.i2c_bus}, address=0x{self.i2c_address:02x}, "
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
    
    def _write_config(self, config: int) -> bool:
        """Write 16-bit config value to config register.
        
        Args:
            config: 16-bit configuration value
            
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            self.logger.debug(f"MOCK: Writing config 0x{config:04x}")
            return True
        
        if not self._connected:
            self.logger.warning("I2C not connected, attempting to reconnect...")
            self._initialize_i2c()
            if not self._connected:
                return False
        
        # Acquire I2C bus lock to prevent concurrent access
        i2c_lock = self._pigpio_conn.get_i2c_bus_lock(self.i2c_bus)
        
        try:
            with i2c_lock:
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
                
                # Write 16-bit config as two bytes (MSB first)
                msb = (config >> 8) & 0xFF
                lsb = config & 0xFF
                
                # Write to config register
                self._pi.i2c_write_i2c_block_data(
                    self._i2c_handle,
                    self.REG_CONFIG,
                    [msb, lsb]
                )
                
                self.logger.debug(f"Config written: 0x{config:04x} (MSB=0x{msb:02x}, LSB=0x{lsb:02x})")
                return True
            
        except Exception as e:
            self.logger.error(f"I2C write config error: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return False
    
    def _read_conversion(self) -> Optional[int]:
        """Read 16-bit conversion result from conversion register.
        
        Returns:
            12-bit ADC value (0-2047 for positive values), or None on error
        """
        if self.mock_mode or self._pigpio_conn.is_mock_mode():
            # Return a mock value in the middle of the range
            mock_value = 1024
            self.logger.debug(f"MOCK: Reading conversion result: {mock_value}")
            return mock_value
        
        if not self._connected:
            self.logger.warning("I2C not connected, attempting to reconnect...")
            self._initialize_i2c()
            if not self._connected:
                return None
        
        # Acquire I2C bus lock to prevent concurrent access
        i2c_lock = self._pigpio_conn.get_i2c_bus_lock(self.i2c_bus)
        
        try:
            with i2c_lock:
                # Ensure we have a valid connection
                self._pi = self._pigpio_conn.get_pi()
                if self._pi is None:
                    self.logger.error("Lost pigpio connection")
                    self._connected = False
                    self._pigpio_conn.increment_error_count()
                    return None
                
                # Verify handle is still valid
                if self._i2c_handle is None or self._i2c_handle < 0:
                    self.logger.error("Invalid I2C handle, attempting to reconnect...")
                    self._initialize_i2c()
                    if not self._connected:
                        return None
                
                # Read 2 bytes from conversion register
                count, data = self._pi.i2c_read_i2c_block_data(
                    self._i2c_handle,
                    self.REG_CONVERSION,
                    2
                )
                
                if count != 2:
                    self.logger.error(f"Expected 2 bytes, got {count}")
                    self._error_count += 1
                    return None
                
                # Combine bytes: result is 12-bit, left-aligned in 16-bit register
                raw = (data[0] << 8) | data[1]
                # Shift right by 4 to get 12-bit value
                value = raw >> 4
                
                self.logger.debug(f"Conversion read: raw=0x{raw:04x}, value={value}")
                return value
            
        except Exception as e:
            self.logger.error(f"I2C read conversion error: {e}")
            self._connected = False
            self._pigpio_conn.increment_error_count()
            return None
    
    def read_adc_raw(self, channel: int) -> Optional[int]:
        """Read raw ADC value from specified channel.
        
        Args:
            channel: ADC channel (0 for AIN0, 1 for AIN1, 2 for AIN2, 3 for AIN3)
        
        Returns:
            12-bit ADC value (0-2047 for positive values), or None on error
        """
        # Validate channel
        if channel not in self.MUX_VALUES:
            self.logger.error(f"Invalid channel {channel}, must be 0-3")
            return None
        
        # Build config with appropriate MUX setting
        mux = self.MUX_VALUES[channel]
        config = self.CONFIG_BASE | mux
        
        # Write config to start conversion
        if not self._write_config(config):
            self._error_count += 1
            return None
        
        # Wait for conversion to complete
        # At 1600 SPS, conversion takes ~625µs, we wait 2ms to be safe
        time.sleep(0.002)
        
        # Read conversion result
        value = self._read_conversion()
        
        if value is None:
            self._error_count += 1
            return None
        
        return value
    
    def read_voltage(self, channel: int) -> Optional[float]:
        """Read voltage from specified channel.
        
        Args:
            channel: ADC channel (0 for AIN0, 1 for AIN1, 2 for AIN2, 3 for AIN3)
        
        Returns:
            Voltage in volts (0.0 to ~4.096V), or None on error
        """
        raw = self.read_adc_raw(channel)
        
        if raw is None:
            return None
        
        # Convert to voltage using PGA_4V scaling
        # With ±4.096V range, LSB = 2mV
        voltage = raw * self.VOLTS_PER_LSB
        
        return voltage
    
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

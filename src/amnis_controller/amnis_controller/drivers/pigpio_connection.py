"""Singleton connection manager for remote pigpio daemon.

This module provides a thread-safe singleton for managing a connection to a
remote Raspberry Pi running the pigpio daemon (pigpiod). This ensures only
one connection is maintained across all driver instances.

The connection can be configured via ROS parameters or constructor arguments.
"""

import threading
import logging
from typing import Optional


class PigpioConnection:
    """Thread-safe singleton for managing remote pigpio connection.
    
    This class ensures only one pigpio connection exists across all driver
    instances, preventing resource exhaustion from multiple connections.
    
    Configuration:
    - host: IP address or hostname of remote Raspberry Pi (default: "localhost")
    - port: pigpiod port (default: 8888)
    - mock_mode: If True, simulate connection without actual hardware
    """
    
    _instance: Optional['PigpioConnection'] = None
    _lock = threading.Lock()
    
    def __new__(cls):
        """Create or return the singleton instance."""
        if cls._instance is None:
            with cls._lock:
                # Double-check locking pattern
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        """Initialize the singleton (only runs once)."""
        # Prevent re-initialization
        if self._initialized:
            return
        
        self._initialized = True
        self._pi = None
        self._host = "localhost"
        self._port = 8888
        self._mock_mode = False
        self._connected = False
        self._error_count = 0
        self._connection_lock = threading.RLock()
        
        self.logger = logging.getLogger('PigpioConnection')
    
    def configure(
        self,
        host: Optional[str] = None,
        port: Optional[int] = None,
        mock_mode: Optional[bool] = None
    ) -> None:
        """Configure connection parameters.
        
        This should be called before connecting. If already connected,
        changes will take effect on next connect() call.
        
        Args:
            host: IP address or hostname of remote Raspberry Pi
            port: pigpiod port number
            mock_mode: If True, simulate connection without hardware
        """
        with self._connection_lock:
            if host is not None:
                self._host = host
            if port is not None:
                self._port = port
            if mock_mode is not None:
                self._mock_mode = mock_mode
            
            self.logger.info(
                f"Configured: host={self._host}, port={self._port}, "
                f"mock_mode={self._mock_mode}"
            )
    
    def connect(self) -> bool:
        """Establish connection to pigpio daemon.
        
        Returns:
            True if connected successfully, False otherwise
        """
        with self._connection_lock:
            # If already connected, return success
            if self._connected and self._pi is not None:
                if not self._mock_mode and self._pi.connected:
                    return True
                elif self._mock_mode:
                    return True
            
            # Disconnect any existing connection
            self._disconnect_internal()
            
            if self._mock_mode:
                self.logger.info("Running in MOCK mode - no actual pigpio connection")
                self._connected = True
                self._pi = None
                return True
            
            try:
                import pigpio
                
                self.logger.info(f"Connecting to pigpiod at {self._host}:{self._port}...")
                self._pi = pigpio.pi(self._host, self._port)
                
                if not self._pi.connected:
                    self.logger.error(
                        f"Failed to connect to pigpiod at {self._host}:{self._port}. "
                        "Make sure pigpiod is running on the remote Raspberry Pi."
                    )
                    self._pi.stop()
                    self._pi = None
                    self._connected = False
                    return False
                
                self._connected = True
                self.logger.info(f"Successfully connected to pigpiod at {self._host}:{self._port}")
                return True
                
            except ImportError:
                self.logger.error(
                    "pigpio library not installed. Install with: pip install pigpio"
                )
                self._connected = False
                return False
            except Exception as e:
                self.logger.error(f"Failed to connect to pigpiod: {e}")
                self._connected = False
                return False
    
    def _disconnect_internal(self) -> None:
        """Internal disconnect without lock (assumes lock is held)."""
        if self._pi is not None and not self._mock_mode:
            try:
                self._pi.stop()
                self.logger.info("Disconnected from pigpiod")
            except Exception as e:
                self.logger.error(f"Error disconnecting from pigpiod: {e}")
        
        self._pi = None
        self._connected = False
    
    def disconnect(self) -> None:
        """Disconnect from pigpio daemon."""
        with self._connection_lock:
            self._disconnect_internal()
    
    def get_pi(self):
        """Get the pigpio.pi instance.
        
        Returns:
            pigpio.pi instance if connected, None otherwise
            
        Note:
            In mock mode, returns None. Callers should check is_mock_mode()
            to determine if they should simulate operations.
        """
        with self._connection_lock:
            if not self._connected:
                self.logger.warning("Not connected, attempting to connect...")
                if not self.connect():
                    return None
            
            # In mock mode, return None (callers should check is_mock_mode)
            if self._mock_mode:
                return None
            
            # Check if connection is still alive
            if self._pi is not None and not self._pi.connected:
                self.logger.warning("Connection lost, attempting to reconnect...")
                if not self.connect():
                    return None
            
            return self._pi
    
    def is_connected(self) -> bool:
        """Check if connected to pigpio daemon.
        
        Returns:
            True if connected (or in mock mode), False otherwise
        """
        with self._connection_lock:
            if self._mock_mode:
                return self._connected
            
            if self._pi is None:
                return False
            
            return self._connected and self._pi.connected
    
    def is_mock_mode(self) -> bool:
        """Check if running in mock mode.
        
        Returns:
            True if in mock mode, False otherwise
        """
        return self._mock_mode
    
    def get_host(self) -> str:
        """Get configured host.
        
        Returns:
            Host string
        """
        return self._host
    
    def get_port(self) -> int:
        """Get configured port.
        
        Returns:
            Port number
        """
        return self._port
    
    def get_error_count(self) -> int:
        """Get the number of connection errors encountered.
        
        Returns:
            Error count
        """
        return self._error_count
    
    def reset_error_count(self) -> None:
        """Reset the error counter."""
        self._error_count = 0
    
    def increment_error_count(self) -> None:
        """Increment the error counter.
        
        This should be called by drivers when they encounter errors.
        """
        self._error_count += 1
    
    @classmethod
    def reset_singleton(cls) -> None:
        """Reset the singleton instance (mainly for testing).
        
        Warning: This will disconnect and destroy the current instance.
        Use with caution in production code.
        """
        with cls._lock:
            if cls._instance is not None:
                cls._instance.disconnect()
                cls._instance = None

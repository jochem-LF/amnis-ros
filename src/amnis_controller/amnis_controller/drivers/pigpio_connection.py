"""Simple module-level connection manager for remote pigpio daemon.

This module provides a shared connection to a remote Raspberry Pi running 
the pigpio daemon (pigpiod). The connection is established when this module 
is first imported.

Configuration via environment variables:
- PIGPIO_HOST: IP address or hostname (default: "localhost")
- PIGPIO_PORT: Port number (default: 8888)
- PIGPIO_MOCK_MODE: Set to 'true' for mock mode (default: "false")
"""

import os
import logging
from typing import Optional

# Module-level logger
logger = logging.getLogger('PigpioConnection')

# Read configuration from environment
PIGPIO_HOST = os.environ.get('PIGPIO_HOST', 'localhost')
PIGPIO_PORT = int(os.environ.get('PIGPIO_PORT', '8888'))
PIGPIO_MOCK_MODE = os.environ.get('PIGPIO_MOCK_MODE', 'false').lower() == 'true'

# Module-level pigpio connection (shared by all drivers)
_pi = None
_connected = False

def _initialize_connection():
    """Initialize the module-level pigpio connection."""
    global _pi, _connected
    
    print("=" * 60)
    print(f"[PigpioConnection] Module Initializing")
    print(f"  Host: {PIGPIO_HOST}")
    print(f"  Port: {PIGPIO_PORT}")
    print(f"  Mock Mode: {PIGPIO_MOCK_MODE}")
    print("=" * 60)
    
    if PIGPIO_MOCK_MODE:
        print(f"[PigpioConnection] Running in MOCK mode")
        logger.info("Running in MOCK mode - no actual pigpio connection")
        _connected = True
        _pi = None
        return
    
    try:
        import pigpio
        
        print("=" * 60)
        print(f"[PigpioConnection] Connecting to pigpiod...")
        print(f"  Target IP: {PIGPIO_HOST}")
        print(f"  Target Port: {PIGPIO_PORT}")
        print("=" * 60)
        
        _pi = pigpio.pi(PIGPIO_HOST, PIGPIO_PORT)
        
        if _pi.connected:
            _connected = True
            print("=" * 60)
            print(f"[PigpioConnection] ✓ SUCCESS: Connected to pigpiod")
            print(f"  Remote Host: {PIGPIO_HOST}:{PIGPIO_PORT}")
            print("=" * 60)
            logger.info(f"✓ Connected to pigpiod at {PIGPIO_HOST}:{PIGPIO_PORT}")
        else:
            _connected = False
            print("=" * 60)
            print(f"[PigpioConnection] ✗ FAILED: Could not connect")
            print(f"  Target: {PIGPIO_HOST}:{PIGPIO_PORT}")
            print(f"  Make sure pigpiod is running: sudo pigpiod")
            print("=" * 60)
            logger.error(f"Failed to connect to pigpiod at {PIGPIO_HOST}:{PIGPIO_PORT}")
            
    except ImportError:
        print("[PigpioConnection] ERROR: pigpio library not installed")
        print("  Install with: pip install pigpio")
        logger.error("pigpio library not installed")
        _connected = False
    except Exception as e:
        print(f"[PigpioConnection] ERROR: {e}")
        logger.error(f"Failed to connect to pigpiod: {e}")
        _connected = False

# Connect immediately when module is imported
print("[PigpioConnection] Module imported - initializing connection...")
_initialize_connection()


def get_pi():
    """Get the shared pigpio.pi instance.
    
    Returns:
        pigpio.pi instance if connected, None otherwise
        
    Note:
        In mock mode, returns None. Callers should check is_mock_mode()
    """
    global _pi, _connected
    
    if PIGPIO_MOCK_MODE:
        return None
    
    # Check if connection is still alive
    if _pi is not None and _pi.connected:
        return _pi
    
    # Try to reconnect if connection was lost
    print("[PigpioConnection] Connection lost, attempting to reconnect...")
    logger.warning("Connection lost, attempting to reconnect...")
    _initialize_connection()
    
    return _pi if _connected else None


def is_connected() -> bool:
    """Check if connected to pigpio daemon.
    
    Returns:
        True if connected (or in mock mode), False otherwise
    """
    if PIGPIO_MOCK_MODE:
        return True
    
    return _pi is not None and _pi.connected


def is_mock_mode() -> bool:
    """Check if running in mock mode.
    
    Returns:
        True if in mock mode, False otherwise
    """
    return PIGPIO_MOCK_MODE


def get_host() -> str:
    """Get configured host.
    
    Returns:
        Host string
    """
    return PIGPIO_HOST


def get_port() -> int:
    """Get configured port.
    
    Returns:
        Port number
    """
    return PIGPIO_PORT


def disconnect():
    """Disconnect from pigpio daemon."""
    global _pi, _connected
    
    if _pi is not None and not PIGPIO_MOCK_MODE:
        try:
            _pi.stop()
            logger.info("Disconnected from pigpiod")
            print("[PigpioConnection] Disconnected from pigpiod")
        except Exception as e:
            logger.error(f"Error disconnecting: {e}")
    
    _pi = None
    _connected = False


# For backward compatibility, provide a dummy class
class PigpioConnection:
    """Dummy class for backward compatibility.
    
    The actual connection is now managed at module level.
    This class just provides access to the module-level functions.
    """
    
    def __init__(self):
        """Initialize (does nothing - connection is already established)."""
        pass
    
    def get_pi(self):
        """Get the pigpio.pi instance."""
        return get_pi()
    
    def is_connected(self) -> bool:
        """Check if connected."""
        return is_connected()
    
    def is_mock_mode(self) -> bool:
        """Check if in mock mode."""
        return is_mock_mode()
    
    def get_host(self) -> str:
        """Get configured host."""
        return get_host()
    
    def get_port(self) -> int:
        """Get configured port."""
        return get_port()
    
    def disconnect(self):
        """Disconnect."""
        disconnect()
    
    def configure(self, host=None, port=None, mock_mode=None):
        """Deprecated - configuration is via environment variables."""
        logger.warning("configure() is deprecated - use environment variables")
    
    def increment_error_count(self):
        """Deprecated."""
        pass

"""Simple helper for pigpio connection configuration.

This module provides the connection configuration from environment variables.

SIMPLIFIED CONNECTION APPROACH:
- Each driver creates its own pigpio.pi() connection at initialization
- No reconnection attempts during operation - if connection fails, you must restart the node
- No shared connection pools or singleton patterns
- Clear error messages guide the user to restart the node if connection is lost

CONFIGURATION:
Set these environment variables before starting the ROS node:
- PIGPIO_HOST: IP address of Raspberry Pi running pigpiod (default: 'localhost')
- PIGPIO_PORT: Port number for pigpiod (default: 8888)
- PIGPIO_MOCK_MODE: Set to 'true' for testing without hardware (default: 'false')

SETUP ON RASPBERRY PI:
1. Install pigpio: sudo apt-get install pigpio python3-pigpio
2. Start pigpiod daemon: sudo pigpiod
3. Verify it's running: sudo systemctl status pigpiod

TROUBLESHOOTING:
- If you see "reconnecting" errors, restart the ROS node
- Ensure pigpiod is running on the Raspberry Pi: sudo pigpiod
- Check network connectivity: ping <PIGPIO_HOST>
- Check firewall allows port 8888
"""

import os

# Read configuration from environment variables
PIGPIO_HOST = os.environ.get('PIGPIO_HOST', 'localhost')
PIGPIO_PORT = int(os.environ.get('PIGPIO_PORT', '8888'))
PIGPIO_MOCK_MODE = os.environ.get('PIGPIO_MOCK_MODE', 'false').lower() == 'true'


def get_config():
    """Get pigpio connection configuration.
    
    Returns:
        Tuple of (host, port, mock_mode)
    """
    return PIGPIO_HOST, PIGPIO_PORT, PIGPIO_MOCK_MODE

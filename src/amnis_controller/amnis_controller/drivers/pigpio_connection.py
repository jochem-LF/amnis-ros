"""Simple helper for pigpio connection configuration.

This module just provides the connection configuration from environment variables.
Each driver creates its own connection - no singleton, no shared state.
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

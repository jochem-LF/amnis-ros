"""EHB (Electro-Hydraulic Brake) driver for brake control via CAN bus.

This module provides a hardware abstraction layer for controlling the vehicle's
electro-hydraulic brake system over CAN bus. It handles low-level CAN communication,
message building/parsing, timing, and error handling.

Compatible with socketcan on Linux systems.
"""

from typing import Optional
import logging
import threading
import time


class EHBDriver:
    """Hardware abstraction for EHB brake system via CAN bus.
    
    This class handles:
    - CAN bus communication (socketcan)
    - Periodic CAN message transmission (0x150, 0x152)
    - CAN response monitoring (0x182)
    - Pressure conversion and message building
    - Error handling and connection monitoring
    
    CAN Protocol:
    - 0x150: Pressure command message (sent at 50Hz)
    - 0x152: System status message (sent at 50Hz)
    - 0x182: EHB feedback message (received from brake system)
    """

    # CAN configuration
    DEFAULT_CAN_INTERFACE = 'socketcan'
    DEFAULT_CAN_CHANNEL = 'can2'
    
    # Message IDs
    MSG_ID_PRESSURE = 0x150
    MSG_ID_STATUS = 0x152
    MSG_ID_FEEDBACK = 0x182
    
    # Timing constants (in seconds)
    TX_PERIOD = 0.02  # 50Hz transmission rate for messages
    RX_TIMEOUT = 0.1  # 100ms timeout for receiving feedback
    ERROR_THRESHOLD = 100  # Error count before connection failure
    ERROR_RECOVERY_CREDIT = 3  # Error count reduction per successful message
    
    # Pressure scaling
    PRESSURE_SCALE = 40.0  # Multiplier for pressure values
    PRESSURE_RESOLUTION = 0.02  # Bar per bit
    
    def __init__(
        self,
        can_channel: str = DEFAULT_CAN_CHANNEL,
        can_interface: str = DEFAULT_CAN_INTERFACE,
        pressure_scale: float = PRESSURE_SCALE,
        mock_mode: bool = False,
    ):
        """Initialize the EHB driver.
        
        Args:
            can_channel: CAN channel name (e.g., 'can0', 'can1', 'can2')
            can_interface: CAN interface type (typically 'socketcan')
            pressure_scale: Scaling factor for pressure commands
            mock_mode: If True, simulate CAN without actual hardware
        """
        self.can_channel = can_channel
        self.can_interface = can_interface
        self.pressure_scale = pressure_scale
        self.mock_mode = mock_mode
        
        self._bus: Optional[object] = None
        self._connected = False
        self._last_pressure = 0.0
        
        # Message counters and switch bits
        self._msg_counter = 0
        self._switch_bit1 = 0
        self._switch_bit2 = 1
        
        # CAN message tracking - simplified approach
        self._last_can_message_time: Optional[float] = None
        self._can_timeout_sec = 0.5  # Error if no message received for 0.5 seconds
        
        # Threading for periodic transmission and reception
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        
        self.logger = logging.getLogger('EHBDriver')
        
        # Try to initialize CAN bus
        self._initialize_can()
        
        # Start transmission and reception threads if connected
        if self._connected or self.mock_mode:
            self._start_threads()
    
    def _initialize_can(self) -> bool:
        """Initialize CAN bus connection.
        
        Returns:
            True if successful, False otherwise
        """
        if self.mock_mode:
            self.logger.info("Running in MOCK mode - no actual CAN communication")
            self._connected = True
            return True
        
        try:
            import can
            self._bus = can.interface.Bus(
                channel=self.can_channel,
                bustype=self.can_interface,
                receive_own_messages=True
            )
            self._connected = True
            self.logger.info(
                f"CAN bus initialized: channel={self.can_channel}, "
                f"interface={self.can_interface}"
            )
            return True
        except ImportError:
            self.logger.error(
                "python-can not installed. Install with: pip install python-can"
            )
            self._connected = False
            return False
        except Exception as e:
            self.logger.error(f"Failed to initialize CAN bus: {e}")
            self._connected = False
            return False
    
    def _start_threads(self) -> None:
        """Start the transmission and reception threads."""
        if self._running:
            return
        
        self._running = True
        
        # Transmission thread (50Hz)
        self._tx_thread = threading.Thread(
            target=self._tx_loop,
            name='ehb-tx',
            daemon=True
        )
        self._tx_thread.start()
        
        # Reception thread (monitors feedback)
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name='ehb-rx',
            daemon=True
        )
        self._rx_thread.start()
        
        self.logger.info("EHB communication threads started")
    
    def _tx_loop(self) -> None:
        """Transmission loop - sends CAN messages at 50Hz."""
        while self._running:
            try:
                self._send_messages()
                time.sleep(self.TX_PERIOD)
            except Exception as e:
                self.logger.error(f"Error in TX loop: {e}")
                time.sleep(self.TX_PERIOD)
    
    def _rx_loop(self) -> None:
        """Reception loop - monitors CAN bus for ANY messages."""
        if self.mock_mode:
            # In mock mode, simulate successful reception
            while self._running:
                time.sleep(self.RX_TIMEOUT)
                with self._lock:
                    self._last_can_message_time = time.time()
            return
        
        while self._running:
            try:
                if not self._connected:
                    time.sleep(self.RX_TIMEOUT)
                    continue
                
                # Receive ANY CAN message (not just feedback)
                message = self._bus.recv(timeout=self.RX_TIMEOUT)
                
                if message is not None:
                    # Update timestamp for ANY received CAN message
                    with self._lock:
                        self._last_can_message_time = time.time()
                        
            except Exception as e:
                self.logger.debug(f"RX error: {e}")
    
    def _send_messages(self) -> None:
        """Send the periodic CAN messages (0x150 and 0x152)."""
        with self._lock:
            pressure = self._last_pressure
            
            # Update counters and switch bits
            self._msg_counter = (self._msg_counter + 1) % 16
            if self._switch_bit1 == 0:
                self._switch_bit1 = 1
                self._switch_bit2 = 0
            else:
                self._switch_bit1 = 0
                self._switch_bit2 = 1
            
            # Build messages
            msg_data_150 = self._build_message_150(pressure, self._switch_bit1, self._switch_bit2)
            msg_data_152 = self._build_message_152(self._msg_counter)
        
        if self.mock_mode:
            self.logger.debug(
                f"MOCK: Sending CAN - Pressure={pressure:.2f} bar, "
                f"Counter={self._msg_counter}, Bits=({self._switch_bit1},{self._switch_bit2})"
            )
            return
        
        if not self._connected:
            self.logger.warning("CAN not connected, skipping transmission")
            return
        
        try:
            import can
            
            # Create CAN messages
            msg_150 = can.Message(
                arbitration_id=self.MSG_ID_PRESSURE,
                data=msg_data_150,
                is_extended_id=False,
                is_rx=False
            )
            msg_152 = can.Message(
                arbitration_id=self.MSG_ID_STATUS,
                data=msg_data_152,
                is_extended_id=False,
                is_rx=False
            )
            
            # Send messages
            self._bus.send(msg_152)
            self._bus.send(msg_150)
            
        except Exception as e:
            self.logger.error(f"Failed to send CAN messages: {e}")
    
    def _build_message_150(self, pressure: float, bit1: int, bit2: int) -> bytearray:
        """Build CAN message 0x150 (pressure command).
        
        This message contains pressure commands for all four wheels and
        dynamic control flags.
        
        Args:
            pressure: Brake pressure in bar (0.0 - 100.0)
            bit1: Switch bit 1 (toggles between 0 and 1)
            bit2: Switch bit 2 (toggles opposite to bit1)
        
        Returns:
            8-byte CAN message data
        """
        # Scale pressure
        pressure_front = int(self.pressure_scale * pressure)
        pressure_rear = int(self.pressure_scale * pressure)
        
        # Convert to pressure value for CAN (14-bit value)
        pressure_val = int(pressure_front / self.PRESSURE_RESOLUTION)
        
        # Initialize message data
        msg_data = bytearray(8)
        
        # Pack data into message (bit-packing from original implementation)
        # Byte 7: Rear right pressure (low byte)
        msg_data[7] = pressure_val & 0xFF
        
        # Byte 6: Rear right pressure (high 6 bits) + bit flags
        msg_data[6] = ((pressure_val & 0x3F00) >> 8)
        msg_data[6] |= (bit1 & 0x01) << 6
        msg_data[6] |= 1 << 7  # DYNAMIC_HR = 1
        
        # Byte 5: Rear left pressure (low byte)
        msg_data[5] = pressure_val & 0xFF
        
        # Byte 4: Rear left pressure (high 6 bits) + bit flags
        msg_data[4] = ((pressure_val & 0x3F00) >> 8)
        msg_data[4] |= (bit2 & 0x01) << 6
        msg_data[4] |= 1 << 7  # DYNAMIC_HL = 1
        
        # Byte 3: Front right pressure (low byte)
        msg_data[3] = pressure_val & 0xFF
        
        # Byte 2: Front right pressure (high 6 bits) + flags
        msg_data[2] = ((pressure_val & 0x3F00) >> 8)
        msg_data[2] |= 1 << 6  # TESTMODE_HY = 1
        msg_data[2] |= 1 << 7  # DYNAMIC_VR = 1
        
        # Byte 1: Front left pressure (low byte)
        msg_data[1] = pressure_val & 0xFF
        
        # Byte 0: Front left pressure (high 6 bits) + flags
        msg_data[0] = ((pressure_val & 0x3F00) >> 8)
        msg_data[0] |= 1 << 6  # TESTMODE_RO = 1
        msg_data[0] |= 1 << 7  # DYNAMIC_VL = 1
        
        return msg_data
    
    def _build_message_152(self, counter: int) -> bytearray:
        """Build CAN message 0x152 (system status).
        
        This message contains system status flags, vehicle information,
        and the message counter.
        
        Args:
            counter: Message counter (0-15)
        
        Returns:
            8-byte CAN message data
        """
        msg_data = bytearray(8)
        
        # System flags (from original implementation)
        FSG_OK = 1
        EHB_AUTONOMOUS = 0
        SET_BRAKELIGHTS = 0
        INF_RFE_RM = 0
        MM_PED_RISE = 0
        KL_61E_P = 1
        BN_SOCS = 0
        DIR_FORW = 1
        BMR_ACT = 0
        DIAG_FSG_ACT = 0
        BRAKE_TYP = 0
        VREF = 0
        ANL_LFT = 0
        BN_NTLF = 0
        VSTAT_A_RM = 0
        N_MOT = 1000
        
        # Byte 0: Counter + flags
        msg_data[0] = counter & 0x0F
        msg_data[0] |= (FSG_OK & 0x01) << 4
        msg_data[0] |= (EHB_AUTONOMOUS & 0x01) << 5
        msg_data[0] |= (SET_BRAKELIGHTS & 0x01) << 6
        msg_data[0] |= (INF_RFE_RM & 0x01) << 7
        
        # Byte 1: Various flags
        msg_data[1] = MM_PED_RISE & 0x01
        msg_data[1] |= (KL_61E_P & 0x01) << 1
        msg_data[1] |= (BN_SOCS & 0x01) << 2
        msg_data[1] |= (DIR_FORW & 0x01) << 4
        msg_data[1] |= (BMR_ACT & 0x01) << 5
        msg_data[1] |= (DIAG_FSG_ACT & 0x01) << 6
        
        # Byte 2: Brake lights + VREF (high bits)
        msg_data[2] = (SET_BRAKELIGHTS & 0x01) << 6
        msg_data[2] |= ((int(VREF / 0.0625) & 0x1F00) >> 8)
        
        # Byte 3: VREF (low byte)
        msg_data[3] = int(VREF / 0.0625) & 0xFF
        
        # Byte 4: Brake type + flags
        msg_data[4] = BRAKE_TYP & 0x0F
        msg_data[4] |= (ANL_LFT & 0x01) << 4
        msg_data[4] |= (BN_NTLF & 0x01) << 5
        msg_data[4] |= (VSTAT_A_RM & 0x01) << 6
        
        # Byte 5: Motor RPM
        msg_data[5] = int(N_MOT / 32) & 0xFF
        
        return msg_data
    
    def set_pressure(self, pressure: float) -> bool:
        """Set brake pressure command.
        
        Args:
            pressure: Desired brake pressure in range [0.0, 1.0]
                     where 0.0 = no braking, 1.0 = full braking
        
        Returns:
            True if command accepted, False otherwise
        """
        # Validate pressure range
        if not (0.0 <= pressure <= 1.0):
            self.logger.error(f"Invalid pressure {pressure}, must be in [0.0, 1.0]")
            return False
        
        with self._lock:
            self._last_pressure = pressure
        
        return True
    
    def is_connected(self) -> bool:
        """Check if CAN bus connection is active and receiving messages.
        
        Returns:
            True if connected and received a message recently (within timeout)
        """
        with self._lock:
            if not self._connected:
                return False
            
            # Check if we received any CAN message recently
            if self._last_can_message_time is None:
                # No messages received yet, but might be starting up
                return True
            
            time_since_last_message = time.time() - self._last_can_message_time
            return time_since_last_message < self._can_timeout_sec
    
    def get_time_since_last_message(self) -> Optional[float]:
        """Get time since last CAN message was received.
        
        Returns:
            Time in seconds since last message, or None if no message received yet
        """
        with self._lock:
            if self._last_can_message_time is None:
                return None
            return time.time() - self._last_can_message_time
    
    def has_can_communication(self) -> bool:
        """Check if we're actively receiving CAN messages.
        
        Returns:
            True if we received a CAN message in the last 0.5 seconds
        """
        return self.is_connected()
    
    def get_last_pressure(self) -> float:
        """Get the last commanded pressure.
        
        Returns:
            Last pressure command [0.0, 1.0]
        """
        with self._lock:
            return self._last_pressure
    
    def stop(self) -> bool:
        """Emergency stop - set brake pressure to 0.
        
        Returns:
            True if successful
        """
        return self.set_pressure(0.0)
    
    def close(self) -> None:
        """Close CAN connection and cleanup.
        
        Stops transmission threads and sends a final zero pressure command.
        """
        try:
            self.logger.info("Stopping EHB driver...")
            
            # Send final stop command
            self.set_pressure(0.0)
            
            # Stop threads
            self._running = False
            
            if self._tx_thread is not None and self._tx_thread.is_alive():
                self._tx_thread.join(timeout=1.0)
            
            if self._rx_thread is not None and self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.0)
            
        except Exception as e:
            self.logger.error(f"Error stopping threads: {e}")
        
        # Close CAN bus
        if self._bus is not None and not self.mock_mode:
            try:
                self._bus.shutdown()
                self.logger.info("CAN bus connection closed")
            except Exception as e:
                self.logger.error(f"Error closing CAN bus: {e}")
        
        self._connected = False


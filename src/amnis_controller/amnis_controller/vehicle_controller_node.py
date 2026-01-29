#!/usr/bin/env python3
"""Vehicle controller node with state machine.

This node controls the vehicle based on a state machine with the following states:
- EHB_ERROR: Emergency state (to be implemented)
- IMMOBILIZED: Vehicle is locked/immobilized (to be implemented)
- MANUAL: Manual mode - all commands are zeroed out
- EXTERNAL: External control mode - joystick commands are forwarded as-is

The initial state is configurable via the 'initial_state' parameter.
State transitions can be added in the future based on vehicle conditions.
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node

from amnis_controller.msg import (
    JoystickCommand, 
    PowertrainCommand, 
    SteerCommand, 
    BrakeCommand, 
    SensorData,
    VehicleState as VehicleStateMsg,
    ModeCommand
)
from amnis_controller.drivers import TransmissionDriver


class VehicleState(Enum):
    """Enum representing the possible vehicle control states."""
    EHB_ERROR = auto()
    IMMOBILIZED = auto()
    MANUAL = auto()
    EXTERNAL = auto()


class VehicleControllerNode(Node):
    """ROS 2 node that manages vehicle control with a state machine.
    
    This node:
    - Subscribes to JoystickCommand messages from the joystick normalizer
    - Manages vehicle state (EHB_ERROR, IMMOBILIZED, MANUAL, EXTERNAL)
    - Publishes vehicle commands based on the current state
    - Controls the external mode relay when transitioning states
    """

    def __init__(self) -> None:
        """Initialize the vehicle controller node."""
        super().__init__('vehicle_controller')

        # Declare parameters
        self.declare_parameter('input_topic', 'vehicle_controller_command')
        self.declare_parameter('sensor_topic', 'sensor_data')
        self.declare_parameter('powertrain_topic', 'powertrain_command')
        self.declare_parameter('steer_topic', 'steer_command')
        self.declare_parameter('brake_topic', 'brake_command')
        self.declare_parameter('vehicle_state_topic', 'vehicle_state')
        self.declare_parameter('mode_command_topic', 'mode_command')
        self.declare_parameter('queue_size', 10)
        
        # Initial vehicle state
        self.declare_parameter('initial_state', 'EXTERNAL')  # MANUAL, EXTERNAL, IMMOBILIZED, EHB_ERROR
        
        # Gas pedal override parameters
        self.declare_parameter('enable_gas_override', True)
        self.declare_parameter('gas_override_raw_min', 500)   # Trigger if raw > this
        self.declare_parameter('gas_override_raw_max', 1500)  # Trigger if raw < this
        self.declare_parameter('verbose_override', True)
        
        # External mode relay configuration
        self.declare_parameter('enable_external_mode_control', True)
        self.declare_parameter('external_mode_pin', 23)  # BCM GPIO 23
        self.declare_parameter('pigpio_host', 'localhost')
        self.declare_parameter('pigpio_port', 8888)
        self.declare_parameter('mock_mode', False)
        
        # Log throttling
        self.declare_parameter('log_throttle_sec', 0.5)
        self.declare_parameter('verbose', True)  # Enable/disable info logging

        # Get parameter values
        self._input_topic = self.get_parameter('input_topic').value
        self._sensor_topic = self.get_parameter('sensor_topic').value
        self._powertrain_topic = self.get_parameter('powertrain_topic').value
        self._steer_topic = self.get_parameter('steer_topic').value
        self._brake_topic = self.get_parameter('brake_topic').value
        self._vehicle_state_topic = self.get_parameter('vehicle_state_topic').value
        self._mode_command_topic = self.get_parameter('mode_command_topic').value
        queue_size = int(self.get_parameter('queue_size').value)
        self._enable_gas_override = self.get_parameter('enable_gas_override').value
        self._gas_override_raw_min = self.get_parameter('gas_override_raw_min').value
        self._gas_override_raw_max = self.get_parameter('gas_override_raw_max').value
        self._verbose_override = self.get_parameter('verbose_override').value
        initial_state_str = self.get_parameter('initial_state').value
        enable_external_mode_control = self.get_parameter('enable_external_mode_control').value
        external_mode_pin = self.get_parameter('external_mode_pin').value
        pigpio_host = self.get_parameter('pigpio_host').value
        pigpio_port = self.get_parameter('pigpio_port').value
        mock_mode = self.get_parameter('mock_mode').value
        self._log_throttle_sec = max(float(self.get_parameter('log_throttle_sec').value), 0.0)
        self.verbose = self.get_parameter('verbose').value

        # Initialize state machine
        try:
            self._current_state: VehicleState = VehicleState[initial_state_str.upper()]
        except KeyError:
            self.get_logger().warning(
                f"Invalid initial_state '{initial_state_str}', defaulting to EXTERNAL"
            )
            self._current_state = VehicleState.EXTERNAL
        
        # Initialize transmission driver for external mode relay control
        self.enable_external_mode_control = enable_external_mode_control
        self.transmission_driver = None
        
        if enable_external_mode_control:
            # We only need the external mode pin, but TransmissionDriver requires all pins
            # The other pins won't be used by this node (they're controlled by powertrain controller)
            self.transmission_driver = TransmissionDriver(
                disable_neutral_pin=17,  # Not used by this node
                enable_reverse_pin=27,   # Not used by this node
                external_mode_pin=external_mode_pin,
                mock_mode=mock_mode,
                pigpio_host=pigpio_host,
                pigpio_port=pigpio_port
            )
            
            if not self.transmission_driver.is_connected():
                self.get_logger().error(
                    "Failed to initialize transmission driver for external mode! "
                    "Running in degraded mode."
                )
            else:
                # Set external mode relay based on initial state
                external_mode = (self._current_state == VehicleState.EXTERNAL)
                self.transmission_driver.set_external_mode(external_mode)
                self.get_logger().info(
                    f"External mode relay control enabled (initial mode: {external_mode})"
                )
        
        # Store the last received joystick command
        self._last_joystick_cmd: Optional[JoystickCommand] = None
        
        # Gas pedal override tracking (no longer needed for raw value checking)
        self._override_active = False  # Track if override already triggered

        # Create publishers for the 3 vehicle subsystems
        self._powertrain_publisher = self.create_publisher(
            PowertrainCommand,
            self._powertrain_topic,
            queue_size
        )
        
        self._steer_publisher = self.create_publisher(
            SteerCommand,
            self._steer_topic,
            queue_size
        )
        
        self._brake_publisher = self.create_publisher(
            BrakeCommand,
            self._brake_topic,
            queue_size
        )
        
        # Create publisher for vehicle state
        self._vehicle_state_publisher = self.create_publisher(
            VehicleStateMsg,
            self._vehicle_state_topic,
            queue_size
        )

        # Create subscription to joystick commands
        self._subscription = self.create_subscription(
            JoystickCommand,
            self._input_topic,
            self._joystick_callback,
            queue_size
        )
        
        # Create subscription to sensor data for gas pedal override
        self._sensor_subscription = self.create_subscription(
            SensorData,
            self._sensor_topic,
            self._sensor_callback,
            queue_size
        )
        
        # Create subscription to mode commands
        self._mode_command_subscription = self.create_subscription(
            ModeCommand,
            self._mode_command_topic,
            self._mode_command_callback,
            queue_size
        )

        # Log startup message
        if self.verbose:
            override_status = f", gas_override={'ON' if self._enable_gas_override else 'OFF'}"
            if self._enable_gas_override:
                override_status += f" (raw range: {self._gas_override_raw_min}-{self._gas_override_raw_max})"
            
            self.get_logger().info(
                f"Vehicle controller ready (state={self._current_state.name}, "
                f"input={self._input_topic}, sensor={self._sensor_topic}, "
                f"outputs=[powertrain={self._powertrain_topic}, "
                f"steer={self._steer_topic}, brake={self._brake_topic}]{override_status})"
            )
        
        # Publish initial state
        self._publish_vehicle_state()

    def _update_state_machine(self) -> None:
        """Update the state machine based on current inputs.
        
        State transitions are currently not implemented - the vehicle stays in
        its initial state. Future implementations may add state transitions based on:
        - EHB error conditions → EHB_ERROR state
        - Immobilization requests → IMMOBILIZED state
        - Manual override requests → MANUAL state
        """
        # For now, maintain the current state
        # Future: Add state transition logic here
        pass

    def _process_command(self, joystick_cmd: JoystickCommand) -> tuple[PowertrainCommand, SteerCommand, BrakeCommand]:
        """Process the joystick command based on the current state.
        
        Args:
            joystick_cmd: The incoming joystick command
            
        Returns:
            Tuple of (powertrain_cmd, steer_cmd, brake_cmd) based on current state
        """
        powertrain_cmd = PowertrainCommand()
        steer_cmd = SteerCommand()
        brake_cmd = BrakeCommand()

        if self._current_state == VehicleState.MANUAL:
            # In MANUAL mode, zero out all commands
            powertrain_cmd.throttle = 0.0
            powertrain_cmd.gear = 0
            steer_cmd.steer = 0.0
            brake_cmd.brake = 0.0
            
        elif self._current_state == VehicleState.EXTERNAL:
            # In EXTERNAL mode, forward joystick commands as-is
            powertrain_cmd.throttle = joystick_cmd.throttle
            powertrain_cmd.gear = joystick_cmd.gear
            steer_cmd.steer = joystick_cmd.steer
            brake_cmd.brake = joystick_cmd.brake
            
        elif self._current_state == VehicleState.IMMOBILIZED:
            # To be implemented - for now, zero everything
            powertrain_cmd.throttle = 0.0
            powertrain_cmd.gear = 0
            steer_cmd.steer = 0.0
            brake_cmd.brake = 0.0
            
        elif self._current_state == VehicleState.EHB_ERROR:
            # To be implemented - for now, zero everything
            powertrain_cmd.throttle = 0.0
            powertrain_cmd.gear = 0
            steer_cmd.steer = 0.0
            brake_cmd.brake = 0.0

        return powertrain_cmd, steer_cmd, brake_cmd

    def _sensor_callback(self, msg: SensorData) -> None:
        """Handle incoming sensor data messages for gas pedal override.
        
        This callback monitors the gas pedal RAW position and automatically
        switches from EXTERNAL to MANUAL mode if the value is in the active
        range (500-1500), indicating the driver is pressing the pedal.
        
        Args:
            msg: SensorData message containing gas pedal and steering wheel positions
        """
        # Only process if gas override is enabled
        if not self._enable_gas_override:
            return
        
        # Get raw gas pedal value
        gas_raw = msg.gas_pedal_raw
        
        # Skip if sensor reading is invalid
        if gas_raw is None or gas_raw < 0:
            return
        
        # Check for manual override only in EXTERNAL mode
        if self._current_state == VehicleState.EXTERNAL:
            # Check if raw value is in the override range
            in_override_range = (gas_raw > self._gas_override_raw_min and 
                               gas_raw < self._gas_override_raw_max)
            
            # Trigger override if in range
            if in_override_range and not self._override_active:
                if self._verbose_override:
                    self.get_logger().warning(
                        f"Gas pedal override detected! Raw value: {gas_raw} "
                        f"(range: {self._gas_override_raw_min}-{self._gas_override_raw_max}) - "
                        "switching to MANUAL mode"
                    )
                
                # Switch to MANUAL mode
                self._current_state = VehicleState.MANUAL
                self._override_active = True
                
                # Disable external mode relay
                if self.enable_external_mode_control and self.transmission_driver is not None:
                    self.transmission_driver.set_external_mode(False)
                    if self._verbose_override:
                        self.get_logger().info("External mode relay disabled")
                
                # Publish the state change
                self._publish_vehicle_state()
        
        # Reset override flag if not in EXTERNAL mode
        if self._current_state != VehicleState.EXTERNAL:
            self._override_active = False
    
    def _mode_command_callback(self, msg: ModeCommand) -> None:
        """Handle incoming mode command messages from the dashboard.
        
        For safety, only MANUAL ↔ EXTERNAL transitions are allowed.
        
        Args:
            msg: ModeCommand message containing target_state
        """
        target_state_str = msg.target_state.upper()
        
        # Validate target state
        try:
            target_state = VehicleState[target_state_str]
        except KeyError:
            self.get_logger().warning(
                f"Invalid mode command: '{msg.target_state}' (valid: MANUAL, EXTERNAL)"
            )
            return
        
        # Safety check: Only allow MANUAL ↔ EXTERNAL transitions
        if target_state not in (VehicleState.MANUAL, VehicleState.EXTERNAL):
            self.get_logger().warning(
                f"Cannot manually switch to {target_state.name} (only MANUAL/EXTERNAL allowed)"
            )
            return
        
        # Check if already in target state
        if self._current_state == target_state:
            if self.verbose:
                self.get_logger().info(f"Already in {target_state.name} mode, ignoring command")
            return
        
        # Perform the state transition
        old_state = self._current_state
        self._current_state = target_state
        
        # Update external mode relay
        if self.enable_external_mode_control and self.transmission_driver is not None:
            external_mode = (target_state == VehicleState.EXTERNAL)
            self.transmission_driver.set_external_mode(external_mode)
        
        # Reset override flag when switching modes
        self._override_active = False
        
        # Log the transition
        self.get_logger().info(
            f"Mode switched: {old_state.name} → {target_state.name}"
        )
        
        # Publish the new state
        self._publish_vehicle_state()
    
    def _publish_vehicle_state(self) -> None:
        """Publish the current vehicle state to the vehicle_state topic."""
        state_msg = VehicleStateMsg()
        state_msg.state = self._current_state.name
        
        # Determine if we can switch to EXTERNAL (only from MANUAL)
        state_msg.can_switch_to_external = (self._current_state == VehicleState.MANUAL)
        
        self._vehicle_state_publisher.publish(state_msg)
    
    def _joystick_callback(self, msg: JoystickCommand) -> None:
        """Handle incoming joystick command messages.
        
        This callback:
        1. Updates the state machine
        2. Processes the command based on current state
        3. Publishes the output commands to 3 separate topics
        4. Logs the activity
        
        Args:
            msg: The incoming JoystickCommand message
        """
        # Store the last command
        self._last_joystick_cmd = msg

        # Update state machine
        self._update_state_machine()

        # Process the command based on current state
        powertrain_cmd, steer_cmd, brake_cmd = self._process_command(msg)

        # Publish the processed commands to their respective topics
        self._powertrain_publisher.publish(powertrain_cmd)
        self._steer_publisher.publish(steer_cmd)
        self._brake_publisher.publish(brake_cmd)

        # Log for debugging
        if self.verbose:
            self.get_logger().info(
                f"[{self._current_state.name}] "
                f"IN(throttle={msg.throttle:.2f}, steer={msg.steer:.2f}, gear={msg.gear}, brake={msg.brake:.2f}) → "
                f"OUT(powertrain: throttle={powertrain_cmd.throttle:.2f}, gear={powertrain_cmd.gear} | "
                f"steer={steer_cmd.steer:.2f} | brake={brake_cmd.brake:.2f})",
                throttle_duration_sec=self._log_throttle_sec
            )

    def destroy_node(self) -> bool:
        """Clean up resources when the node shuts down."""
        # Disable external mode relay before shutdown
        if self.enable_external_mode_control and self.transmission_driver is not None:
            self.get_logger().info("Disabling external mode relay...")
            self.transmission_driver.set_external_mode(False)
            self.transmission_driver.close()
        
        if getattr(self, '_subscription', None) is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        
        if getattr(self, '_sensor_subscription', None) is not None:
            self.destroy_subscription(self._sensor_subscription)
            self._sensor_subscription = None
        
        if getattr(self, '_mode_command_subscription', None) is not None:
            self.destroy_subscription(self._mode_command_subscription)
            self._mode_command_subscription = None
        
        if getattr(self, '_powertrain_publisher', None) is not None:
            self.destroy_publisher(self._powertrain_publisher)
            self._powertrain_publisher = None
            
        if getattr(self, '_steer_publisher', None) is not None:
            self.destroy_publisher(self._steer_publisher)
            self._steer_publisher = None
            
        if getattr(self, '_brake_publisher', None) is not None:
            self.destroy_publisher(self._brake_publisher)
            self._brake_publisher = None
        
        if getattr(self, '_vehicle_state_publisher', None) is not None:
            self.destroy_publisher(self._vehicle_state_publisher)
            self._vehicle_state_publisher = None
        
        return super().destroy_node()


def main(args=None) -> None:
    """Main entry point for the vehicle controller node."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = VehicleControllerNode()

    try:
        # Start the main loop
        rclpy.spin(node)
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


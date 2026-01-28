#!/usr/bin/env python3
"""Vehicle controller node with state machine.

This node controls the vehicle based on a state machine with the following states:
- EHB error: Emergency state (to be implemented)
- immobilized: Vehicle is locked/immobilized (to be implemented)
- manual: Manual mode - all commands are zeroed out (for now)
- external: External control mode - joystick commands are forwarded as-is

The state machine manages transitions between states based on safety and mode buttons.
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node

from amnis_controller.msg import JoystickCommand, PowertrainCommand, SteerCommand, BrakeCommand


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
    - Manages vehicle state (EHB error, immobilized, manual, external)
    - Publishes vehicle commands based on the current state
    - Handles state transitions based on safety and mode buttons
    """

    def __init__(self) -> None:
        """Initialize the vehicle controller node."""
        super().__init__('vehicle_controller')

        # Declare parameters
        self.declare_parameter('input_topic', 'vehicle_controller_command')
        self.declare_parameter('powertrain_topic', 'powertrain_command')
        self.declare_parameter('steer_topic', 'steer_command')
        self.declare_parameter('brake_topic', 'brake_command')
        self.declare_parameter('queue_size', 10)
        
        # Simulated button inputs (will be replaced with actual hardware later)
        self.declare_parameter('safety_button', False)
        self.declare_parameter('mode_button', False)
        
        # Log throttling
        self.declare_parameter('log_throttle_sec', 0.5)
        self.declare_parameter('verbose', True)  # Enable/disable info logging

        # Get parameter values
        self._input_topic = self.get_parameter('input_topic').value
        self._powertrain_topic = self.get_parameter('powertrain_topic').value
        self._steer_topic = self.get_parameter('steer_topic').value
        self._brake_topic = self.get_parameter('brake_topic').value
        queue_size = int(self.get_parameter('queue_size').value)
        self._log_throttle_sec = max(float(self.get_parameter('log_throttle_sec').value), 0.0)
        self.verbose = self.get_parameter('verbose').value

        # Initialize state machine to MANUAL state
        self._current_state: VehicleState = VehicleState.MANUAL
        
        # Store the last received joystick command
        self._last_joystick_cmd: Optional[JoystickCommand] = None

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

        # Create subscription to joystick commands
        self._subscription = self.create_subscription(
            JoystickCommand,
            self._input_topic,
            self._joystick_callback,
            queue_size
        )

        # Log startup message
        if self.verbose:
            self.get_logger().info(
                f"Vehicle controller ready (state={self._current_state.name}, "
                f"input={self._input_topic}, outputs=[powertrain={self._powertrain_topic}, "
                f"steer={self._steer_topic}, brake={self._brake_topic}])"
            )

    def _get_button_states(self) -> tuple[bool, bool]:
        """Get the current button states.
        
        For now, this reads from parameters. In the future, this will read
        from actual hardware buttons.
        
        Returns:
            Tuple of (safety_button, mode_button) where True = pressed
        """
        # Re-read parameters to allow runtime changes
        safety_button = bool(self.get_parameter('safety_button').value)
        mode_button = bool(self.get_parameter('mode_button').value)
        return safety_button, mode_button

    def _update_state_machine(self) -> None:
        """Update the state machine based on current inputs.
        
        State transitions:
        - manual → external: when mode_button=1 and safety_button=0
        - external → manual: when mode_button=0
        - immobilized and EHB_ERROR: to be implemented
        """
        safety_button, mode_button = self._get_button_states()
        previous_state = self._current_state

        if self._current_state == VehicleState.MANUAL:
            # Transition from MANUAL to EXTERNAL
            if mode_button and not safety_button:
                self._current_state = VehicleState.EXTERNAL
                
        elif self._current_state == VehicleState.EXTERNAL:
            # Transition from EXTERNAL back to MANUAL
            if not mode_button:
                self._current_state = VehicleState.MANUAL
                
        elif self._current_state == VehicleState.IMMOBILIZED:
            # To be implemented
            pass
            
        elif self._current_state == VehicleState.EHB_ERROR:
            # To be implemented
            pass

        # Log state changes
        if self._current_state != previous_state:
            if self.verbose:
                self.get_logger().info(
                    f"State transition: {previous_state.name} → {self._current_state.name} "
                    f"(safety={safety_button}, mode={mode_button})"
                )

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
            safety_button, mode_button = self._get_button_states()
            self.get_logger().info(
                f"[{self._current_state.name}] "
                f"IN(throttle={msg.throttle:.2f}, steer={msg.steer:.2f}, gear={msg.gear}, brake={msg.brake:.2f}) → "
                f"OUT(powertrain: throttle={powertrain_cmd.throttle:.2f}, gear={powertrain_cmd.gear} | "
                f"steer={steer_cmd.steer:.2f} | brake={brake_cmd.brake:.2f}) "
                f"[safety={safety_button}, mode={mode_button}]",
                throttle_duration_sec=self._log_throttle_sec
            )

    def destroy_node(self) -> bool:
        """Clean up resources when the node shuts down."""
        if getattr(self, '_subscription', None) is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        
        if getattr(self, '_powertrain_publisher', None) is not None:
            self.destroy_publisher(self._powertrain_publisher)
            self._powertrain_publisher = None
            
        if getattr(self, '_steer_publisher', None) is not None:
            self.destroy_publisher(self._steer_publisher)
            self._steer_publisher = None
            
        if getattr(self, '_brake_publisher', None) is not None:
            self.destroy_publisher(self._brake_publisher)
            self._brake_publisher = None
        
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


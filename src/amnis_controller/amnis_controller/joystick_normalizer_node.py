#!/usr/bin/env python3
"""Joystick normalization node for Xbox controller inputs.

This module implements a ROS 2 node that:
1. Listens to raw joystick/Xbox controller input from the 'joy' package
2. Normalizes the axis values to a consistent [-1.0, 1.0] range
3. Publishes vehicle-friendly command values in a custom message so other nodes
    can use them for control

The normalization is especially important for Xbox controllers because:
- Analog sticks report [-1, 1] by default
- Triggers report [0, 1] by default (we convert to [-1, 1] for consistency)
- Values may have drift that we filter with a deadzone
"""
from __future__ import annotations

from typing import Sequence, Set

import rclpy
# rclpy is the Python client library for ROS 2 - it gives us access to ROS 2 core functionality
from rclpy.node import Node
# Node is the base class for all ROS 2 nodes - our class will inherit from it
from sensor_msgs.msg import Joy
# Joy is a message type defined by sensor_msgs that represents joystick state
# It contains arrays of axis values and button states and is only used as input

from amnis_controller.msg import JoystickCommand
# JoystickCommand is our custom message type published to downstream vehicle logic

from amnis_controller.normalization import axes_to_command_values, normalize_axes
# Import helper functions that handle axis normalization and command mapping


class JoystickNormalizerNode(Node):
    """ROS 2 node that republishes Xbox controller inputs normalized to [-1, 1].
    
    A ROS 2 Node is a program that:
    - Subscribes to one or more topics (receives data)
    - Publishes to one or more topics (sends data)
    - Can have services and timers
    - Has parameters that can be configured at launch time
    
    This node specifically:
    - Subscribes to /joy (raw joystick data from the joy driver)
    - Processes the data through normalize_axes()
    - Publishes JoystickCommand messages (throttle/steer/gear/brake/cmd)
    """

    def __init__(self) -> None:
        """Initialize the node.
        
        The __init__ method runs once when the node starts. It:
        1. Calls the parent Node class constructor with a node name
        2. Declares parameters (configuration values) with defaults
        3. Retrieves those parameters from the ROS 2 parameter server
        4. Creates a publisher (to send messages)
        5. Creates a subscription (to receive messages)
        6. Logs a startup message
        """
        # Call the parent Node class constructor, giving this node a name
        # ROS 2 will use this name to identify the node in the system
        super().__init__('joystick_normalizer')

        # Declare parameters with default values
        # Parameters allow you to configure the node without changing code
        # They can be set via launch files, command line, or the parameter server
        
        # The input topic where we receive raw joy messages from the joy driver
        self.declare_parameter('input_topic', '/joy')

        # The output topic where we publish JoystickCommand messages
        # Other nodes (like a vehicle controller) will listen to this
        self.declare_parameter('output_topic', 'vehicle_controller_command')

        # Which axes are triggers? Xbox controllers have triggers at axes 2 and 5
        # Triggers are special because they report [0, 1] instead of [-1, 1]
        # Axis 2 = left trigger (brake), Axis 5 = right trigger (throttle)
        self.declare_parameter('trigger_axes', [2, 5])

        # Deadzone: any value with absolute magnitude below this is treated as 0
        # This prevents controller drift (sticks slowly moving on their own)
        self.declare_parameter('deadzone', 0.05)

        # Queue size for the publisher and subscriber
        # This is the maximum number of messages to buffer if the receiving code is slow
        self.declare_parameter('queue_size', 10)

        # Log throttling: how often to print log messages (in seconds)
        # Without this, we'd print a message for EVERY joystick input (potentially 100+ per second!)
        # Throttling reduces console spam while still letting us see what's happening
        self.declare_parameter('log_throttle_sec', 0.5)
        
        # Enable/disable info logging
        self.declare_parameter('verbose', True)

        # Retrieve the parameter values from the parameter server
        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        
        # Handle the trigger_axes parameter flexibly (it might be a list, tuple, or single value)
        trigger_axes_param = self.get_parameter('trigger_axes').value
        if isinstance(trigger_axes_param, (list, tuple, set)):
            trigger_axes_iterable = trigger_axes_param
        elif trigger_axes_param is None:
            trigger_axes_iterable = []
        else:
            trigger_axes_iterable = [trigger_axes_param]
        # Convert to a set of integers, filtering out negative indices
        self._trigger_axes: Set[int] = {int(axis) for axis in trigger_axes_iterable if int(axis) >= 0}
        
        # Store deadzone, ensuring it's not negative
        self._deadzone = max(float(self.get_parameter('deadzone').value), 0.0)
        
        # Get queue size parameter
        queue_size_param = int(self.get_parameter('queue_size').value)
        
        # Store log throttle time, ensuring it's not negative
        self._log_throttle_sec = max(float(self.get_parameter('log_throttle_sec').value), 0.0)
        
        # Get verbose parameter
        self.verbose = self.get_parameter('verbose').value

        # Track the current gear state for toggle behavior
        self._current_gear: int = 0

        # Create a publisher
        # This allows this node to send messages to other ROS 2 nodes
        # Arguments: message type (JoystickCommand), topic name, queue size
        self._publisher = self.create_publisher(JoystickCommand, self._output_topic, queue_size_param)

        # Create a subscription (listener)
        # This allows this node to receive messages from other ROS 2 nodes
        # Arguments: message type (Joy), topic name, callback function, queue size
        # When a message arrives on self._input_topic, self._joy_callback will be called
        self._subscription = self.create_subscription(
            Joy,
            self._input_topic,
            self._joy_callback,
            queue_size_param,
        )

        # Log a startup message so we know the node started successfully
        # This also shows the configuration, useful for debugging
        if self.verbose:
            self.get_logger().info(
                f"Joystick normalizer ready (input={self._input_topic}, output={self._output_topic}, "
                f"triggers={sorted(self._trigger_axes)}, deadzone={self._deadzone:.3f})"
            )


    def _joy_callback(self, msg: Joy) -> None:
        """Handle incoming joystick messages.

        This method is called automatically by ROS 2 every time a message arrives
        on the input_topic. It:
        1. Receives the raw Joy message from the joy driver
        2. Normalizes the axis values using our normalize_axes() helper
        3. Maps the normalized axes to vehicle-friendly command values
        4. Publishes a JoystickCommand message for downstream nodes
        5. Logs what was received (throttled to avoid spam)

        Args:
            msg (Joy): The incoming message from the joy driver containing axes and buttons
        """
        # Call our normalization helper function
        # This converts axes to [-1, 1] range and applies deadzone
        normalized_axes = normalize_axes(msg.axes, self._trigger_axes, self._deadzone)

        # Convert the normalized axes into semantic vehicle command values
        # Pass buttons and current_gear for toggle behavior
        throttle, steer, gear, brake, cmd = axes_to_command_values(
            normalized_axes,
            msg.buttons,
            current_gear=self._current_gear
        )

        # Update the stored gear state
        self._current_gear = gear

        # Populate the custom JoystickCommand message
        command_msg = JoystickCommand()
        command_msg.throttle = throttle
        command_msg.steer = steer
        command_msg.gear = gear
        command_msg.brake = brake
        # Command value is reserved for future button mappings (0 = no command)
        command_msg.cmd = cmd

        # Publish the command so other nodes (e.g., the vehicle controller) can consume it
        self._publisher.publish(command_msg)

        # Log the normalized values and resulting command for debugging/monitoring
        if self.verbose:
            axes_as_str = ', '.join(f'{value:+.3f}' for value in normalized_axes)
            buttons_as_str = ', '.join(str(button) for button in msg.buttons)
            self.get_logger().info(
                f'axes=[{axes_as_str}] buttons=[{buttons_as_str}] -> '
                f'throttle={throttle:.2f}, steer={steer:.2f}, gear={gear}, '
                f'brake={brake:.2f}, cmd={cmd}',
                throttle_duration_sec=self._log_throttle_sec,
            )


    def destroy_node(self) -> bool:
        """Clean up resources when the node shuts down.
        
        This method is called when the node is shutting down (e.g., Ctrl+C).
        It properly closes the publisher and subscriber to avoid resource leaks.
        
        Returns:
            bool: True if cleanup was successful
        """
        # Safely check if the subscription exists and destroy it
        # This tells ROS 2 to stop listening to the input_topic
        if getattr(self, '_subscription', None) is not None:
            self.destroy_subscription(self._subscription)
            self._subscription = None
        
        # Safely check if the publisher exists and destroy it
        # This tells ROS 2 to stop the publisher
        if getattr(self, '_publisher', None) is not None:
            self.destroy_publisher(self._publisher)
            self._publisher = None
        
        # Call the parent class cleanup
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    """Entry point for the node when run as a standalone executable.
    
    This function is called when you run:
        ros2 run amnis_controller joystick_normalizer_node
    
    It:
    1. Initializes ROS 2 (rclpy.init)
    2. Creates an instance of our node
    3. Spins the node (runs the main loop that processes callbacks)
    4. Handles shutdown gracefully (Ctrl+C will exit cleanly)
    
    Args:
        args: Command-line arguments passed to ROS 2 (usually None, ROS handles them)
    """
    # Initialize ROS 2
    # This must be called once per process before using any ROS 2 features
    rclpy.init(args=args)
    
    # Create our node instance
    node = JoystickNormalizerNode()
    
    try:
        # Start the main loop
        # rclpy.spin() runs indefinitely, calling our callbacks when messages arrive
        # It keeps the node alive until interrupted (Ctrl+C)
        rclpy.spin(node)
    finally:
        # This runs when the node is shutting down (always, even if there was an error)
        # Clean up the node
        node.destroy_node()
        
        # Shut down ROS 2
        rclpy.shutdown()


# This allows the module to be run directly
# When a node is started with "ros2 run", this is called automatically via setup.py entry_points
__all__ = ['JoystickNormalizerNode']


if __name__ == '__main__':
    main()


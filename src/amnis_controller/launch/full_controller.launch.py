"""

This launch file starts:
0. joy_node - Publishes raw joystick data from hardware device
1. joystick_normalizer_node - Normalizes joystick inputs
2. vehicle_controller_node - Manages vehicle state machine and command filtering
3. steer_controller_node - Controls steering motor via I2C H-bridge
4. brake_controller_node - Controls EHB brake system via CAN bus
5. powertrain_controller_node - Controls throttle via PWM on GPIO
6. sensor_reader_node - Reads gas pedal and steering wheel from ADC
7. topic_aggregator_node - Bridges ROS topics to a WebSocket frontend
8. Firefox browser - Opens dashboard in fullscreen/kiosk mode
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with joystick, controller, and steer nodes."""
    
    # game_controller_node - reads from joystick hardware
    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[{
            'device_id': 0,  # Usually 0 for /dev/input/js0
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,  # Hz
        }]
    )
    
    # Joystick normalizer node
    joystick_node = Node(
        package='amnis_controller',
        executable='joystick_normalizer_node',
        name='joystick_normalizer',
        output='screen',
        parameters=[{
            'input_topic': '/joy',
            'output_topic': 'vehicle_controller_command',
            'trigger_axes': [2],
            'deadzone': 0.05,
            'log_throttle_sec': 0.5,
            'verbose': False,  # Logging disabled
        }]
    )
    
    # Vehicle controller node with state machine
    controller_node = Node(
        package='amnis_controller',
        executable='vehicle_controller_node',
        name='vehicle_controller',
        output='screen',
        parameters=[{
            'input_topic': 'vehicle_controller_command',
            'sensor_topic': 'sensor_data',
            'powertrain_topic': 'powertrain_command',
            'steer_topic': 'steer_command',
            'brake_topic': 'brake_command',
            'safety_button': False,  # Simulated - will be hardware later
            'mode_button': True,    # Simulated - will be hardware later
            # External mode relay configuration (remote GPIO via pigpio)
            'enable_external_mode_control': True,
            'external_mode_pin': 4,        # BCM GPIO 4 (physical pin 7)
            'pigpio_host': '192.168.10.2',     # IP of Raspberry Pi running pigpiod
            'pigpio_port': 8888,            # Default pigpiod port
            'mock_mode': False,             # Set True for testing without hardware
            # Gas pedal override configuration
            'enable_gas_override': True,    # Enable automatic EXTERNAL→MANUAL on gas pedal press
            'gas_override_threshold': 0.15, # 15% change triggers override
            'verbose_override': True,       # Log override events
            'log_throttle_sec': 0.5,
            'verbose': False,  # Logging disabled
        }]
    )
    
    # Steer controller node - controls H-bridge via remote I2C (pigpio)
    steer_controller_node = Node(
        package='amnis_controller',
        executable='steer_controller_node',
        name='steer_controller',
        output='screen',
        parameters=[{
            'input_topic': 'steer_command',
            'diagnostic_topic': 'steer_diagnostics',
            'i2c_bus': 1,                    # I2C bus on Raspberry Pi (typically bus 1)
            'i2c_address': 0x58,             # H-bridge I2C address
            'max_power': 100,                # Maximum speed percentage
            # Pigpio connection configuration
            'pigpio_host': '192.168.10.2',      # IP of Raspberry Pi running pigpiod
            'pigpio_port': 8888,             # Default pigpiod port
            'mock_mode': False,              # Set True for testing without hardware
            'command_timeout_sec': 0.5,
            'deadzone': 0.05,
            'update_rate_hz': 20.0,
            'steer_to_power_scale': 100.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': False,  # Logging disabled
        }]
    )
    
    # Brake controller node - controls EHB via CAN buss
    brake_controller_node = Node(
        package='amnis_controller',
        executable='brake_controller_node',
        name='brake_controller',
        output='screen',
        parameters=[{
            'input_topic': 'brake_command',
            'diagnostic_topic': 'brake_diagnostics',
            'can_channel': 'can1',           # CAN channel (e.g., can0, can1, can2)
            'can_interface': 'socketcan',    # CAN interface type
            'pressure_scale': 40.0,          # Pressure scaling factor
            'mock_mode': False,              # Set True for testing without hardware
            'command_timeout_sec': 0.5,
            'deadzone': 0.01,
            'update_rate_hz': 10.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': False,  # Logging disabled
        }]
    )
    
    # Powertrain controller node - controls throttle via PWM and transmission via relays
    powertrain_controller_node = Node(
        package='amnis_controller',
        executable='powertrain_controller_node',
        name='powertrain_controller',
        output='screen',
        parameters=[{
            'input_topic': 'powertrain_command',
            'diagnostic_topic': 'powertrain_diagnostics',
            # PWM throttle configuration (remote GPIO via pigpio)
            'pwm_pin': 22,                   # BCM GPIO 22 (physical pin 15)
            'pwm_frequency': 1000,           # PWM frequency in Hz
            'max_throttle': 1.0,             # Maximum throttle (0.0-1.0, set lower for testing)
            # Pigpio connection configuration
            'pigpio_host': '192.168.10.2',      # IP of Raspberry Pi running pigpiod
            'pigpio_port': 8888,             # Default pigpiod port
            # Transmission relay configuration (gear control only - external mode in vehicle_controller)
            'enable_transmission_control': True,
            'disable_neutral_pin': 12,       # BCM GPIO 12 (physical pin 32)
            'enable_reverse_pin': 5,         # BCM GPIO 5 (physical pin 29)
            # General configuration
            'mock_mode': False,              # Set True for testing without hardware
            'command_timeout_sec': 0.5,
            'deadzone': 0.01,
            'update_rate_hz': 20.0,
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': False,  # Logging disabled
        }]
    )
    
    # Sensor reader node - reads gas pedal and steering wheel from ADC
    sensor_reader_node = Node(
        package='amnis_controller',
        executable='sensor_reader_node',
        name='sensor_reader',
        output='screen',
        parameters=[{
            'output_topic': 'sensor_data',
            'diagnostic_topic': 'sensor_diagnostics',
            # Hardware configuration
            'i2c_bus': 1,                    # I2C bus on Raspberry Pi
            'i2c_address': 0x48,             # ADS1015L I2C address (ADDR to GND)
            # Pigpio connection configuration
            'pigpio_host': '192.168.10.2',      # IP of Raspberry Pi running pigpiod
            'pigpio_port': 8888,             # Default pigpiod port
            'mock_mode': False,              # Set True for testing without hardware
            # Calibration (set these after calibrating your potentiometers)
            # Leave at defaults (0, 2047) for full range, or set specific values
            'gas_pedal_min': 0,              # Raw ADC min value for gas pedal
            'gas_pedal_max': 2047,           # Raw ADC max value for gas pedal
            'steering_wheel_min': 0,         # Raw ADC min value for steering wheel
            'steering_wheel_max': 2047,      # Raw ADC max value for steering wheel
            'auto_calibrate': False,         # Set True to auto-calibrate at startup
            'calibration_duration_sec': 10.0,# Auto-calibration duration
            # Control
            'update_rate_hz': 10.0,          # Sensor reading rate (lower to reduce I2C bus contention)
            # Diagnostics
            'publish_diagnostics': True,
            'log_throttle_sec': 1.0,
            'verbose': True,                 # Set to True to enable debug logging
        }]
    )
    
    # Topic aggregator node - exposes ROS topics over WebSockets
    aggregator_node = Node(
        package='amnis_controller',
        executable='topic_aggregator_node',
        name='topic_aggregator',
        output='screen',
        parameters=[{
            'topic_poll_interval': 2.0,
            'include_hidden_topics': False,
            'ignored_topics': ['/parameter_events', '/rosout'],
            'websocket_host': '0.0.0.0',
            'websocket_port': 8765,
            'update_frequency_hz': 30.0,  # Throttle updates to 0.5Hz (2 seconds) to reduce resource usage must be float
        }]
    )
    
    # Firefox browser - opens dashboard in kiosk mode (fullscreen)
    # Construct absolute path to dashboard.html (assumes it's in workspace root)
    workspace_dir = Path(os.getcwd())
    dashboard_path = workspace_dir / 'dashboard.html'
    dashboard_url = f'file://{dashboard_path.absolute()}'
    
    firefox_process = ExecuteProcess(
        cmd=['firefox', '--kiosk', dashboard_url],
        output='screen',
        name='firefox_dashboard'
    )

    return LaunchDescription([
        game_controller_node,
        joystick_node,
        controller_node,
        steer_controller_node,
        brake_controller_node,
        powertrain_controller_node,
        sensor_reader_node,
        aggregator_node,
        firefox_process,
    ])


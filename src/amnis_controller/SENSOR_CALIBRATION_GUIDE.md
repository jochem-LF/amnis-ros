# Sensor Calibration Guide

This guide will help you calibrate the gas pedal and steering wheel potentiometers connected to the ADS1015L ADC.

## Overview

The sensor input node reads analog voltages from two potentiometers:
- **Gas Pedal** on AIN0: normalized to [0.0, 1.0]
- **Steering Wheel** on AIN1: normalized to [-1.0, 1.0]

The normalization requires knowing the actual voltage range of each sensor, which varies based on your potentiometer wiring and physical setup.

## Hardware Setup

### ADS1015L Connections
- **VDD**: 3.3V or 5V power supply
- **GND**: Ground
- **SCL**: I2C clock (connect to RPi I2C SCL)
- **SDA**: I2C data (connect to RPi I2C SDA)
- **ADDR**: Connect to GND for address 0x48 (default)

### Sensor Connections
- **Gas Pedal Potentiometer**:
  - Pin 1: GND
  - Pin 2 (wiper): AIN0
  - Pin 3: 3.3V
  
- **Steering Wheel Potentiometer**:
  - Pin 1: GND
  - Pin 2 (wiper): AIN1
  - Pin 3: 3.3V

## Calibration Procedure

### Step 1: Initial Test Run

1. Build and source your ROS2 workspace:
```bash
cd ~/amnis-ros
colcon build --packages-select amnis_controller
source install/setup.bash
```

2. Run the sensor node alone with verbose logging:
```bash
ros2 run amnis_controller sensor_input_node --ros-args \
  -p verbose:=true \
  -p mock_mode:=false \
  -p pigpio_host:=192.168.10.2
```

3. Watch the output - you'll see voltage readings like:
```
[sensor_input]: Gas: 0.823V → 0.323 | Steering: 1.654V → 0.467
```

### Step 2: Record Gas Pedal Range

1. **With the gas pedal NOT pressed** (rest position):
   - Record the voltage reading for Gas
   - Example: `0.523V`
   - This is your `gas_min_voltage`

2. **Press the gas pedal FULLY**:
   - Record the voltage reading for Gas
   - Example: `2.987V`
   - This is your `gas_max_voltage`

### Step 3: Record Steering Wheel Range

1. **Turn the steering wheel FULL LEFT**:
   - Record the voltage reading for Steering
   - Example: `0.312V`
   - This is your `steer_min_voltage`

2. **Turn the steering wheel FULL RIGHT**:
   - Record the voltage reading for Steering
   - Example: `3.187V`
   - This is your `steer_max_voltage`

### Step 4: Update Launch File

Edit `launch/full_controller.launch.py` and update the calibration parameters:

```python
# In the sensor_input_node parameters section:
'gas_min_voltage': 0.523,    # Your measured rest voltage
'gas_max_voltage': 2.987,    # Your measured full press voltage
'steer_min_voltage': 0.312,  # Your measured full left voltage
'steer_max_voltage': 3.187,  # Your measured full right voltage
```

### Step 5: Verify Calibration

1. Rebuild and restart the node:
```bash
colcon build --packages-select amnis_controller
source install/setup.bash
ros2 run amnis_controller sensor_input_node --ros-args \
  -p verbose:=true \
  -p gas_min_voltage:=0.523 \
  -p gas_max_voltage:=2.987 \
  -p steer_min_voltage:=0.312 \
  -p steer_max_voltage:=3.187
```

2. Test the ranges:
   - **Gas pedal at rest**: Should show `→ 0.000` (or very close to 0.0)
   - **Gas pedal fully pressed**: Should show `→ 1.000` (or very close to 1.0)
   - **Steering full left**: Should show `→ -1.000` (or very close to -1.0)
   - **Steering center**: Should show `→ 0.000` (approximately)
   - **Steering full right**: Should show `→ 1.000` (or very close to 1.0)

## Published Topics

The sensor node publishes to:
- `/sensor/gas_pedal` (std_msgs/Float32): Gas pedal position [0.0, 1.0]
- `/sensor/steering_wheel` (std_msgs/Float32): Steering position [-1.0, 1.0]

You can monitor these topics:
```bash
ros2 topic echo /sensor/gas_pedal
ros2 topic echo /sensor/steering_wheel
```

## Integration with Vehicle Controller

The sensor topics are separate from the joystick input, allowing you to:

1. **Use sensors only**: Modify the vehicle controller to subscribe to `/sensor/*` topics
2. **Use joystick only**: Keep current configuration
3. **Switch between inputs**: Add a mode selector in the vehicle controller
4. **Blend inputs**: Create logic to combine or prioritize different inputs

## Troubleshooting

### No ADC Readings
- Check I2C connection: `i2cdetect -y 1` on the Raspberry Pi
- Verify pigpiod is running: `sudo systemctl status pigpiod`
- Check pigpio_host parameter matches RPi IP address
- Verify ADC power supply (VDD) is connected

### Incorrect Voltage Range
- Verify potentiometer connections (Pin 1=GND, Pin 3=3.3V)
- Check if potentiometer is wired backwards (swap Pin 1 and Pin 3)
- Measure voltage with multimeter to confirm ADC readings

### Noisy Readings
- Add capacitors (0.1µF) between each AIN and GND
- Reduce update_rate_hz in parameters
- Use shielded cables for sensor connections
- Keep sensor wires away from high-current motor wires

### Inverted Output
- If gas pedal shows 1.0 at rest and 0.0 when pressed:
  - Swap gas_min_voltage and gas_max_voltage values
- If steering shows inverted direction:
  - Swap steer_min_voltage and steer_max_voltage values

## Mock Mode Testing

For testing without hardware:

```bash
ros2 run amnis_controller sensor_input_node --ros-args \
  -p mock_mode:=true \
  -p verbose:=true
```

In mock mode, the ADC will return simulated readings (~1.65V on all channels).

## Advanced Configuration

### Change I2C Address
If your ADDR pin is not connected to GND:
```python
'adc_i2c_address': 0x49,  # ADDR to VDD
# or 0x4A (ADDR to SDA), or 0x4B (ADDR to SCL)
```

### Adjust Update Rate
For faster/slower readings:
```python
'update_rate_hz': 50.0,  # 50Hz instead of default 20Hz
```

### Custom Topic Names
```python
'gas_pedal_topic': '/my_custom/gas',
'steering_topic': '/my_custom/steering',
```

## Next Steps

After calibration:
1. Test sensor readings are accurate and stable
2. Integrate sensor topics with your vehicle controller
3. Add safety limits (e.g., maximum throttle)
4. Consider adding sensor diagnostics/monitoring
5. Test the full system in a safe environment

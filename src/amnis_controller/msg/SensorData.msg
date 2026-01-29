# Analog sensor readings from ADS1015L ADC
# All values are normalized to range [0.0, 1.0]

float32 gas_pedal      # Gas pedal position (0.0 = released, 1.0 = fully pressed)
float32 steering_wheel # Steering wheel potentiometer (0.0 = min, 1.0 = max)

# Raw ADC values (12-bit, 0-2047) for diagnostics
int16 gas_pedal_raw
int16 steering_wheel_raw

# Calibration status
bool calibrated        # True if calibration values have been set


"""Hardware and experiment settings. Values are starting points, not tuning results."""

# Sensor numbers 1..13, exactly as printed on the sensor PCB.
SENSOR_PINS = ("C5", "C0", "A6", "C4", "B1", "C3", "A0",
               "A7", "C2", "C1", "B0", "A4", "A1")
CHANNELS = tuple(range(1, 14))
# None asks at startup which printed end lies on Romi's LEFT. After verifying,
# set True for channel 1 on the left or False for channel 13 on the left.
CHANNEL_1_ON_LEFT = None
CTRL_PINS = ("B2", "H1")
ADC_TIMER = 6                 # TIM1/2 encoders, TIM3 PWM, TIM6 ADC only.
ADC_HZ = 5000                # 13 * 5000 = 65000 conversions/s.
ADC_SAMPLES = 10             # About 2 ms blocking acquisition per update.
CALIBRATION_BATCHES = 50
MIN_SPAN = 10.0              # ADC counts; verify against measured noise/contrast.
CALIBRATION_FILE = "line_calibration.json"


class Settings:
    motor_period_ms = 10
    motor_priority = 2
    line_period_ms = 20
    line_priority = 1
    ui_period_ms = 0
    ui_priority = 0
    samples_after_initial = 100
    battery_voltage = 9.0     # Update to your measured battery reference.
    voltage_min = -4.5
    voltage_max = 4.5
    kp_max = 0.1             # Entry bounds only, NOT a stable-gain guarantee.
    ki_max = 1.0
    kff_max = 0.1
    setpoint_max = 10000
    preparation_min_ms = 300
    preparation_timeout_ms = 3000
    rest_velocity_limit = 50.0
    rest_samples_required = 10
    line_duration_s = 10.0
    line_max_duration_s = 30.0
    line_log_period_ms = 100  # Control remains 20 ms; log every fifth update.
    wheel_speed_max = 3000 # counts/s; reduce to your experimentally tested range.
    correction_max = 1000   # counts/s; also capped to keep both requests in range.
    line_stale_ms = 100      # Stop if the outer task stops refreshing requests.
    line_gain_max = 100.0    # (counts/s)/mm, entry bound only.


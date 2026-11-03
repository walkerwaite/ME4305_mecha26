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
ADC_SAMPLES = 8             # About 2 ms blocking acquisition per update.
CALIBRATION_BATCHES = 50
MIN_SPAN = 10.0              # ADC counts; verify against measured noise/contrast.
CALIBRATION_FILE = "line_calibration.json"


class Settings:
    motor_period_ms = 40
    motor_priority = 3

    # Include motion period and priority
    motion_period_ms = 40
    motion_priority = 2
    line_period_ms = motion_period_ms

    ui_period_ms = 40
    ui_priority = 1
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
    line_duration_s = 20.0
    line_max_duration_s = 30.0
    line_log_period_ms = 250  # Control remains 20 ms; changed this from 100 to 250 ms for storage reasons
    wheel_speed_max = 3000 # counts/s; reduce to your experimentally tested range.
    correction_max = 1000   # counts/s; also capped to keep both requests in range.
    line_stale_ms = 100      # Stop if the outer task stops refreshing requests.
    line_gain_max = 100.0    # (counts/s)/mm, entry bound only.

    m_per_encoder_count = 0.000048611  # meters per encoder count
    
    white_weight_max = 0.009  # Maximum weight for the white line in the line-following algorithm.
    all_black_weight_min = 0.99  # Minimum weight for the black line in the line-following algorithm.
    line_detect_weight = 0.01 # Weight for detecting the line in the line-following algorithm.

    white_clear_ms = 90 # Time in milliseconds to consider the white line as cleared.

    # IMU configuration
    imu_stale_ms = 200

    # Lane change configuration
    maneuver_enabled = True

    # +1 = move left, -1 = move right, 0 = no lane change
    lane_direction = 1

    # Measured from the grid with 25mm grid, 
    # four intervals = 100mm, 0.1m
    line_spacing_m = 0.1

    # Begin turn with base speeds and yaw rates
    maneuver_base_speed = 500.0  # counts/s
    yaw_rate_ref_rad_s = 0.6 # magnitude only
    yaw_rate_gain = 200 # (counts/s)/(rad/s)

    # Existing line following before departure
    initial_follow_time_s = 1.0

    # Background table displacement
    turnback_displacement_m = 0.03 # meters

    # Safety:
    maneuver_timeout_s = 15.0 # seconds, stop if not completed in this time


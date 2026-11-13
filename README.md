# ME 4305 Romi Firmware: Lab 0x08

**Checkpoint:** `Lab 0x08 CHECKPOINT: Rename README.txt to README.md`.  
**Entry point:** `main.py` on the Romi MicroPython board.

This checkpoint combines the former line-following and IMU tasks into one `TaskMotion`, retaining the individual wheel PI-FF controllers and USB test interface. It updates the task periods and priorities such that starvation of the user task no longer occurs.

## Source map

|Files|Responsibility|
|-|-|
|`main.py`, `romi_config.py`|Hardware initialization, scheduler, shared data, configuration, safe shutdown.|
|`taskmotion.py`|IMU updates, line following, lane-change states, wheel-speed requests.|
|`taskmotor.py`|USB commands, calibration prompts, and data collection.|
|`taskuser.py`|Two independent motor tasks.|
|`motordriver.py`, `encoder.py`, `closedloop.py`|PWM/direction driver, quadrature feedback, and PI-FF voltage controller.|
|`imudriver.py`, `linesensor.py`|BNO055 IMU and 13-channel QTR-HD-13A reflectance sensor.|
|`imu_calibration.py`, `line_calibration.py`|Calibration JSON validation and storage.|
|`PC_auto_plotter_CL.py`, `PC_auto_plotter_line.py`|PC-only motor/line test capture, CSV output, and plots.|



## Setup and startup

1. Use compatible MicroPython firmware with `pyb`, `micropython`, and `ulab`; install the course-supplied `cotask.py`. Copy the MCU files with the exact names above, including `main.py` and `romi_config.py`. For PC tests, install Python, `pyserial`, and `matplotlib` (`pip install pyserial matplotlib`). Keep both PC scripts and `romi_config.py` together: the line plotter imports the closed-loop plotter, and both use the shared encoder scale.
2. Verify **left encoder** TIM1/PA8–PA9 and **right encoder** TIM2/PA15–PB3. **Left motor:** TIM3 CH2/PB5 PWM, PC7 direction, PC6 sleep. **Right motor:** TIM3 CH1/PB4 PWM, PB10 direction, PC8 sleep. PWM is 20 kHz. The line array uses **TIM6**, the 13 ADC pins in `romi_config.SENSOR_PINS` (channels 1–13 in order), and PB2/PH1 emitter controls; supply **3.3 V AVDD** and common ground. The BNO055 uses **I2C(1), 100 kHz, address `0x28`, PC9 reset**; check I2C1 SCL/SDA wiring.
3. Raise the wheels and connect a USB serial terminal (e.g., PuTTY) to the board's COM port. Reset the board. Motors are disabled during initialization, and the BNO055 restores `imu_calibration.json` or performs startup calibration with motors disabled.
4. At the line-sensor prompt, select **`c`** to capture black/white calibration, **`u`** to reuse a valid calibration under unchanged conditions, or **`m`** for motor-only tests. For a new capture, confirm the array's leftmost printed channel and follow the all-black/all-white placement prompts.
5. Use **`h`** for help, **`k`** for PI–FF gains and wheel speed, **`n`** for line-follower settings, **`s`** to request a stop, and **`e`** to exit safely. Check motion direction with wheels raised before track operation.

## Configuration and interfaces

* **Robot-specific calibration:** `imu_calibration.json` stores BNO055's 22-byte offsets and configuration; `line_calibration.json` stores 13 black/white ADC readings, sensor orientation, and acquisition settings. Retain these with the checkpoint, but recheck or regenerate on another robot after remounting sensors or after changing lighting/track conditions.
* **Robot-specific settings:** Verify actual battery voltage (`battery_voltage = 9.0` V is a reference), left/right PI–FF gains, sensor orientation (`CHANNEL_1_ON_LEFT`), encoder distance scale (`0.000048611` m/count), line thresholds, lane-change geometry, speeds, and turn signs. Default gains are starting values, not validated tuning.
* **Scheduling:** Left and right `TaskMotor` run every **40 ms** (priority 2) and `TaskMotion` every **60 ms** (priority 1). `TaskUser` is continuously eligible (priority 0). Line data is **logged every 250 ms**, independent of the motion period.
* **Units:** `Encoder` gives position in **counts** and angular velocity in **rad/s**, using the configured 1440 encoder counts per wheel revolution (`rad/s = counts/s × 2π / 1440`). `ClosedLoop.run(vel, K_P, v_sp, K_I, K_FF, dt)` returns **volts** (`dt` in s; `K_P`, `K_FF` in V/(rad/s); `K_I` in V/rad). `MotorDriver.set_effort()` instead takes signed **PWM percent**. `LineSensor.read()` returns centroid in **mm** (positive left, `None` if undefined); the IMU returns heading in **rad** and yaw rate in **rad/s** (both positive left). Verify actual encoder and IMU signs on the robot.
* **Task handoff:** `TaskMotion` alone writes `LineRun.target = [left, right]` in rad/s; each `TaskMotor` owns its wheel's encoder, controller, motor, and ready/stopped feedback. Line correction is `left = base - K_line*centroid`, `right = base + K_line*centroid`, where `K_line` is (rad/s)/mm. `lane_direction = +1` means left; `-1` means right.

## Reproducing verification

Keep a copy of the old firmware for before/after comparison. Record calibration, gains, battery reference, timing, and track conditions for each test.

1. **Startup and stopping:** Boot from a separate complete copy; verify motors remain disabled during calibration. Try `s` during motion and `e` for shutdown; then restart and repeat a run.
2. **Motor regression (wheels raised):** In the gain menu use `k` → `p/i/f` → `v` → `l` or `r`, or configure `TESTS` and `SERIAL_PORT` in `PC_auto_plotter_CL.py` and run it on the PC. Save velocity/voltage response data and confirm limits.
3. **Integrated regression (track):** Run paired straight motion (`k` → `v` → `b`), then line following/lane change (`n` → `b/p/t` → `g`). Alternatively configure `MODE`, gains, duration, and port in `PC_auto_plotter_line.py`. **Close PuTTY before running a PC plotter.** Check steering, line reacquisition, stop bar, stopping, exported CSV, and scheduler profile.

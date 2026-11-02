"""Lab 0x07: BNO055 calibration storage and startup, before scheduling.

Pattern follows line_calibration.py from Lab 0x06, but stores the BNO055's
own 22-byte offsets and radii, not QTR black/white measurements. Keep
imu_calibration.json separate from line_calibration.json. The driver handles
LSB/MSB register reads and writes; this file handles JSON validation/storage.

Both Romi motors MUST remain disabled while startup calibration is underway.
"""

import json
from time import ticks_ms, ticks_diff, sleep_ms

CAL_FILE = "imu_calibration.json"


def validate_calibration(calibration, configuration):
    # Match the physical mounting and all IMU settings before reusing offsets.
    if not isinstance(calibration, dict):
        raise ValueError("IMU calibration must be a dictionary")
    if (type(calibration.get("version")) is not int or
            calibration["version"] != 1 or
            calibration.get("configuration") != configuration):
        raise ValueError("Saved IMU calibration configuration does not match")
    offsets = calibration.get("offsets")
    if not isinstance(offsets, list) or len(offsets) != 22:
        raise ValueError("IMU calibration must contain exactly 22 bytes")
    for value in offsets:
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError("IMU offsets must be integers from 0 to 255")
    return calibration


def save_calibration(filename, imu):
    # Collect sensor data first: gyro and accel must each be calibrated to 3.
    calibration = {"version": 1, "configuration": imu.configuration(),
                   "offsets": imu.get_calibration()}
    validate_calibration(calibration, imu.configuration())
    with open(filename, "w") as file:
        json.dump(calibration, file)
    return calibration


def load_calibration(filename, imu):
    # Read/validate BEFORE modifying any BNO055 registers. The driver then
    # restores the bytes in register order (LSB, MSB) under CONFIG mode.
    with open(filename, "r") as file:
        calibration = json.load(file)
    return validate_calibration(calibration, imu.configuration())


def wait_for_calibration(imu, timeout_ms=300000, interval_ms=500):
    # Startup ONLY. Keep still for gyro calibration; hold six orientations
    # for accelerometer calibration, observing status (system, gyro, accel, mag).
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < timeout_ms:
        status = imu.calibration_status()
        if status[1] == 3 and status[2] == 3:
            return True  # IMUPLUS does not need magnetometer calibration.
        print("IMU calib (system,gyro,accel,mag):", status)
        sleep_ms(interval_ms)
    return imu.is_calibrated()


def startup_calibration(imu, filename=CAL_FILE):
    """Restore saved offsets or calibrate and save; call BEFORE scheduling.

    Returns 'loaded' or 'calibrated'. A timeout or I2C failure raises and
    must STOP application startup; never enable motors in that case.
    """
    try:
        calibration = load_calibration(filename, imu)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("IMU calibration file missing/invalid:", error)
        calibration = None

    if calibration is not None:
        try:
            imu.set_calibration(calibration["offsets"])
            print("Restored IMU calibration")
            return "loaded"
        except (ValueError, RuntimeError) as error:
            # A valid JSON file is not sufficient if the BNO055 rejects it.
            print("IMU profile restore failed; recalibrating:", error)

    print("Keep BOTH motors disabled while calibrating the IMU")
    print("Hold still for gyro; move through six stable orientations for accel")
    if not wait_for_calibration(imu):
        raise RuntimeError("IMU calibration timeout: do not start motors")
    save_calibration(filename, imu)
    print("Saved new IMU calibration")
    return "calibrated"


# Example STARTUP sequence, before setting up your task scheduler:
# from imudriver import BNO055
# from imu_calibration import startup_calibration
# imu = BNO055(i2c, reset_pin=rst, mount_id="romi_flat_z_up")
# startup_calibration(imu)  # Motors must be disabled; creates JSON if needed.
# Now start the scheduler. Scheduled tasks ONLY read:
# heading_rad = imu.get_heading()
# yaw_rate_rad_s = imu.get_yaw_rate()

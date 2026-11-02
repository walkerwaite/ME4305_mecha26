"""Runtime IMU measurement task."""

from time import ticks_us
from imudriver import BNO055
from math import isfinite


class TaskIMU:
    def __init__(self, imu, data):
        self.imu = imu
        self.data = data

    def run(self):
        while True:
            # Keep a previous fault visible to TaskLine.
            # Restarting the application creates a fresh IMUData object.
            if self.data.fault:
                self.data.valid = False
                yield 0
                continue

            try:
                heading = self.imu.get_heading()
                yaw_rate = self.imu.get_yaw_rate()

                if not (isfinite(heading) and isfinite(yaw_rate)):
                    raise ValueError("Nonfinite IMU measurement")

                self.data.heading_rad = heading
                self.data.yaw_rate_rad_s = yaw_rate
                self.data.updated_us = ticks_us()
                self.data.valid = True

            except (OSError, ValueError, RuntimeError) as error:
                self.data.valid = False
                self.data.fault = str(error) or "IMU read failed"

            yield 0
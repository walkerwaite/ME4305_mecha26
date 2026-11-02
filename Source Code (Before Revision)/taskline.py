"""Outer wheel-request task with a bounded, IMU-assisted lane change.

This is a replacement draft for taskline.py.  It preserves the Lab 0x06
motor handshake: TaskLine alone writes LineRun.target, while the two
TaskMotor objects remain in TRACK state and close their inner PI-FF loops.

Runtime IMU ownership belongs to TaskIMU.  This task receives only its
shared IMUData object; it never calls the BNO055 driver directly.

Required LineSensor additions:
    read_weights()            -> updates and returns calibrated weights
    centroid_from_weights()   -> computes a centroid from those weights

Required Settings additions are listed in the comments in _prepare().
"""

from time import ticks_us, ticks_diff
from math import isfinite, sin, pi
import romi_config as cfg



S0_IDLE = 0
S1_PREP = 1
S2_FOLLOW_START = 2
S3_DEPART = 3
S4_TURN_BACK = 4
S5_FOLLOW_NEW = 5
S6_STOP = 6


class TaskLine:
    """Outer steering and lane-change state-machine owner.

    Positive centroid, positive yaw rate, and lane_direction = +1 all mean
    a Romi left turn / leftward departure.  Set lane_direction to -1 for a
    rightward lane change.
    """

    def __init__(self, sensor, imu_data, run, data, print_flag, settings,
                 gain_banks):
        self.sensor = sensor
        self.imu_data = imu_data
        self.run_request = run
        self.data = data
        self.print_flag = print_flag
        self.settings = settings
        self.gain_banks = gain_banks
        self.state = S0_IDLE
        self.started = False

    def _prepare(self):
        r, s = self.run_request, self.settings

        self.data.reset()
        self.started = False
        self.mode = r.request
        self.base = r.straight_speed if self.mode == "straight" else r.base.value
        self.gain, self.duration = r.gain.value, r.duration.value

        if not all(isfinite(v) for v in (self.base, self.gain, self.duration)):
            raise ValueError("Nonfinite line settings")
        if not 0 < self.base < s.wheel_speed_max:
            raise ValueError("Base speed must lie inside configured wheel range")
        if not 0 <= self.gain < s.line_gain_max or not 0 < self.duration <= s.line_max_duration_s:
            raise ValueError("Invalid line gain or duration")
        if self.mode == "line" and not self.sensor.calibrated:
            raise ValueError("Calibrate before line following")
        if self.mode not in ("line", "straight"):
            raise ValueError("Unknown paired-wheel mode")

        # New Settings values, all fixed configuration values; no UI is needed.
        # maneuver_enabled, lane_direction, initial_follow_time_s,
        # yaw_rate_ref_rad_s, yaw_rate_gain, turnback_displacement_m,
        # maneuver_timeout_s, m_per_encoder_count, white_weight_max,
        # line_detect_weight, all_black_weight_min, imu_stale_ms.
        self.maneuver_enabled = (self.mode == "line" and
                                 bool(getattr(s, "maneuver_enabled", False)))
        if self.maneuver_enabled:
            self.direction = s.lane_direction
           
            if self.direction not in (-1, 1):
                raise ValueError("lane_direction must be +1 or -1")
            
            values = (s.initial_follow_time_s, s.yaw_rate_ref_rad_s,
                      s.yaw_rate_gain, s.turnback_displacement_m,
                      s.maneuver_timeout_s, s.m_per_encoder_count,
                      s.white_weight_max, s.line_detect_weight,
                      s.all_black_weight_min, s.imu_stale_ms)
            
            
            if not all(isfinite(value) for value in values):
                raise ValueError("Nonfinite maneuver setting")
            if (s.initial_follow_time_s < 0 or s.yaw_rate_ref_rad_s <= 0 or
                    s.yaw_rate_gain <= 0 or s.turnback_displacement_m <= 0 or
                    s.maneuver_timeout_s <= 0 or s.m_per_encoder_count <= 0):
                raise ValueError("Invalid positive maneuver setting")
            if not (0 <= s.white_weight_max < s.line_detect_weight <= 1 and
                    0 < s.all_black_weight_min <= 1):
                raise ValueError("Invalid normalized line thresholds")

        self.limit = min(s.correction_max, self.base,
                         s.wheel_speed_max - self.base)
        
        r.fault, r.running = "", False
        r.ready[:] = [False, False]
        r.stopped[:] = [True, True]
        r.target[:] = [0.0, 0.0]
        r.velocity[:] = [0.0, 0.0]
        r.voltage[:] = [0.0, 0.0]
        self.sensor.max_read_us = 0

        self.data.metadata = {
            "loop_type": self.mode,
            "control_structure": "outer P / IMU yaw P / inner PI-FF",
            "base_counts_per_s": self.base,
            "outer_gain_counts_per_s_per_mm": self.gain,
            "correction_limit_counts_per_s": self.limit,
            "maneuver_enabled": self.maneuver_enabled,
            "motor_period_ms": s.motor_period_ms,
            "line_period_ms": s.line_period_ms,
            "imu_stale_ms": getattr(s, "imu_stale_ms", None),
            "calibration_file": cfg.CALIBRATION_FILE,
            "centroid_convention": "positive left, zero centered; nan undefined",
        }
        if self.maneuver_enabled:
            self.data.metadata.update({
                "lane_direction": self.direction,
                "yaw_rate_ref_rad_s": s.yaw_rate_ref_rad_s,
                "yaw_rate_gain_counts_per_s_per_rad_s": s.yaw_rate_gain,
                "turnback_displacement_m": s.turnback_displacement_m,
                "m_per_encoder_count": s.m_per_encoder_count,
            })
        for side, gains in self.gain_banks.items():
            for key in ("K_P", "K_I", "K_FF"):
                self.data.metadata[side + "_" + key] = getattr(gains, key).value

        if (not isfinite(s.white_clear_ms) or s.white_clear_ms < s.line_period_ms):
            raise ValueError("White interval must span at least one line period")

        self._check_imu(ticks_us())

        self.prep_start = ticks_us()
        r.active = True
        self.state = S1_PREP

    def _elapsed_ms(self, now):
        return ticks_diff(now, self.start_us) / 1000

    def _set_wheel_requests(self, correction, now):
        """The one path by which this task commands the two wheel loops."""
        # This is for line following wheel corrections
        r = self.run_request
        bounded = min(self.limit, max(-self.limit, correction))
        r.target[0] = self.base - bounded
        r.target[1] = self.base + bounded
        
        r.updated_us = ticks_us()
        r.running = True

    def _line_steering(self, centroid, now):
        if centroid is None:
            raise ValueError("Undefined centroid while line following")
        self._set_wheel_requests(self.gain * centroid, now)

    def _check_imu(self, now):
        if self.imu_data.fault:
            raise RuntimeError("IMU task error: " + self.imu_data.fault)
        if not self.imu_data.valid:
            raise RuntimeError("No valid IMU measurement available")
        if ticks_diff(now, self.imu_data.updated_us) > self.settings.imu_stale_ms * 1000:
            raise RuntimeError("IMU measurement is stale")
        if not (isfinite(self.imu_data.heading_rad) and
                isfinite(self.imu_data.yaw_rate_rad_s)):
            raise RuntimeError("Nonfinite IMU measurement")

    def _yaw_rate_steering(self, reference_rad_s, now):
        # Controlling the robot based on the yaw rate error
        self._check_imu(now)
        error = reference_rad_s - self.imu_data.yaw_rate_rad_s
        self._set_wheel_requests(self.settings.yaw_rate_gain * error, now)

    def _update_lateral_displacement(self, now):
        """Integrate encoder forward speed projected by IMU relative heading."""
        self._check_imu(now)
        dt_s = ticks_diff(now, self.lateral_last_us) / 1000000
        self.lateral_last_us = now
       
        if dt_s < 0:
            raise RuntimeError("Invalid lateral-displacement timestamp")

        # Average two wheel velocities to estimate forward speed

        # v_k = (v_left + v_right) / 2
        forward_counts_s = 0.5 * (self.run_request.velocity[0] +
                                  self.run_request.velocity[1])

        # Convert forward speed from encoder counts per second to meters per second
        # v_k (m/s) = v_k (counts/s) * m_per_encoder_count
        forward_m_s = forward_counts_s * self.settings.m_per_encoder_count

        # Compute the relative heading of the robot with respect to its starting heading
        # psi
        relative_heading = (self.imu_data.heading_rad - self.heading_start + pi) % (2 * pi) - pi

        # Update the lateral displacement based on the forward speed and relative heading
        # d_k+1 = d_k + v_k*psi*dt
        self.lateral_y_m += forward_m_s * relative_heading * dt_s

    def _read_pattern(self, now):
        """Read once, then return pattern flags and that sample's centroid."""
        # One acquisition, no centroid calc yet
        self.sensor.read(calculate_centroid=False)

        weights = self.sensor.weights
        s = self.settings

        # Determine if all weights indicate a white line, black line, or ordinary line
        all_white = all(float(w) <= s.white_weight_max for w in weights)

        all_black = all(float(w) >= s.all_black_weight_min for w in weights)
        
        ordinary_line = (not all_white and not all_black and any(float(w) >= s.line_detect_weight for w in weights))

        # Require continuous white during lane change
        if self.state in (S3_DEPART, S4_TURN_BACK):
            if all_white:
                if self.white_since_us is None:
                    self.white_since_us = now

                if ticks_diff(now, self.white_since_us) >= s.white_clear_ms * 1000:
                    self.seen_white = True

            else:
                self.white_since_us = None

        return all_white, ordinary_line, all_black    

    def _record(self, now, centroid=None):
        elapsed = self._elapsed_ms(now)
        if elapsed >= self.next_log_ms:
            self.data.record(elapsed, centroid, self.sensor.total_weight,
                             self.run_request)
            self.next_log_ms = elapsed + self.settings.line_log_period_ms

    def _begin_departure(self, now):
        self._check_imu(now)

        self.heading_start = self.imu_data.heading_rad
        self.lateral_y_m = 0.0
        self.lateral_last_us = now
        self.maneuver_start_us = now
        self.seen_white = False
        self.white_since_us = None
        self.state = S3_DEPART

    def _maneuver_timed_out(self, now):
        return (ticks_diff(now, self.maneuver_start_us) >=
                self.settings.maneuver_timeout_s * 1000000)

    def _stop(self, status, reason=""):
        r = self.run_request
        r.stop, r.running = True, False
        r.target[:] = [0.0, 0.0]
        self.data.status, self.data.reason = status, reason
        self.data.metadata.setdefault("loop_type", r.request or "line")
        if self.started:
            self.data.metadata["actual_duration_ms"] = self._elapsed_ms(ticks_us())
        self.state = S6_STOP

    def _follow_start(self, now):
        if self.mode == "straight":
            self._set_wheel_requests(0.0, now)
            self._record(now)
            return

        _, ordinary_line, _ = self._read_pattern(now)

        if not ordinary_line:
            self._stop("failed", "Ordinary line not found")
            return

        centroid = self.sensor.read(refresh=False)
        self._line_steering(centroid, now)
        self._record(now, centroid)

        if (self.maneuver_enabled and
                self._elapsed_ms(now) >= self.settings.initial_follow_time_s * 1000):
            self._begin_departure(now)

    def _depart(self, now):
        # Updates the continuous white timer
        self._read_pattern(now)

        self._update_lateral_displacement(now)

        self._yaw_rate_steering(self.direction * self.settings.yaw_rate_ref_rad_s,
                                now)
        self._record(now)

        # direction*y is positive for a correctly directed left OR right shift.
        # Check if the lateral displacement has reached the turn-back threshold.
        if self.direction * self.lateral_y_m >= self.settings.turnback_displacement_m:
            self.state = S4_TURN_BACK

        elif self._maneuver_timed_out(now):
            self._stop("failed", "Lane-change timeout before turn-back")

    def _turn_back(self, now):
        _, ordinary_line, _ = self._read_pattern(now)

        self._update_lateral_displacement(now)

        if self.seen_white and ordinary_line:
            self.state = S5_FOLLOW_NEW

            # Hand steering back immediately using same sample
            centroid = self.sensor.read(refresh=False)
            self._line_steering(centroid, now)
            self._record(now, centroid)

        elif self._maneuver_timed_out(now):
            self._stop("failed", "Lane-change timeout: target line not found")

        else:
            self._yaw_rate_steering(-self.direction * self.settings.yaw_rate_ref_rad_s,
                                now)
            self._record(now)


    def _follow_new(self, now):
        _, ordinary_line, all_black = self._read_pattern(now)

        # No centroid has been calculated for this sample:
        if all_black:
            self._stop("complete", "Transverse all-black stop bar")
            return

        if not ordinary_line:
            self._stop("failed", "Ordinary line not found")
            return

        centroid = self.sensor.read(refresh=False)
        self._line_steering(centroid, now)
        self._record(now, centroid)

    def _run_active_state(self, now):
            r = self.run_request

            # Preserve a fault reported by either motor task.
            if r.fault:
                self._stop("failed", r.fault)
                return

            if r.stop:
                self._stop("stopped", "User stop")
                return

            # Check IMU health throughout the maneuver.
            if self.maneuver_enabled:
                self._check_imu(now)

            if (
                self.state in (S3_DEPART, S4_TURN_BACK)
                and self._maneuver_timed_out(now)
            ):
                self._stop("failed", "Maneuver timeout")
                return

            if self._elapsed_ms(now) >= self.duration * 1000:
                self._stop("complete", "Run duration reached")
                return

            # Use one connected if/elif chain.
            if self.state == S2_FOLLOW_START:
                self._follow_start(now)

            elif self.state == S3_DEPART:
                self._depart(now)

            elif self.state == S4_TURN_BACK:
                self._turn_back(now)

            elif self.state == S5_FOLLOW_NEW:
                self._follow_new(now)

            else:
                raise RuntimeError(
                    "Invalid active TaskLine state: " + str(self.state)
                )

    def run(self):
        while True:
            r = self.run_request
            try:
                if self.state == S0_IDLE:

                    if r.request is not None and not self.print_flag.print_flag:
                        self._prepare()

                elif self.state == S1_PREP:
                    if self.maneuver_enabled and not r.stop and not r.fault:
                        self._check_imu(ticks_us())

                    if r.fault:
                        self._stop("failed", r.fault)
                        return
                    elif r.stop:
                        self._stop("stopped", "User stop")
                        return


                    elif all(r.ready):
                        self.start_us = ticks_us()
                        self.next_log_ms = 0.0
                        self.started = True
                        self.data.status = "running"
                        self.state = S2_FOLLOW_START
                        self._run_active_state(self.start_us)

                    elif ticks_diff(ticks_us(), self.prep_start) > (
                            self.settings.preparation_timeout_ms + 100) * 1000:
                        self._stop("failed", "Wheel preparation timeout")

                elif self.state in (S2_FOLLOW_START, S3_DEPART,
                                    S4_TURN_BACK, S5_FOLLOW_NEW):
                    self._run_active_state(ticks_us())

                elif self.state == S6_STOP:
                    # Wait for BOTH inner-loop motor owners to acknowledge disable.
                    if all(r.stopped):
                        r.active, r.request = False, None
                        self.data.metadata["expected_samples"] = self.data.count
                        self.data.metadata["adc_max_read_us"] = self.sensor.max_read_us
                        self.print_flag.print_flag = True
                        self.state = S0_IDLE

                else:
                    raise RuntimeError("Invalid TaskLine state")

            except (ValueError, TypeError, RuntimeError, OSError) as error:
                self._stop("failed", str(error))

            yield self.state

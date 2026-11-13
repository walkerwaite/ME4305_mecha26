"""Outer P steering task and finite two-wheel experiment coordinator.

TaskLine owns the sensor. It never operates motors/encoders directly.
Positive centroid requests right wheel faster, turning toward Romi's left.
Straight mode disables the outer loop for equal-speed loaded wheel tests.
"""
from time import ticks_us, ticks_diff
from math import isfinite
import romi_config as cfg

S0_IDLE = 0
S1_PREP = 1
S2_FOLLOW = 2
S3_STOP = 3


class TaskLine:
    def __init__(self, sensor, run, data, print_flag, settings, gain_banks):
        self.sensor, self.run_request, self.data = sensor, run, data
        self.print_flag, self.settings, self.gain_banks = print_flag, settings, gain_banks
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
            raise ValueError('Nonfinite line settings')
        if not 0 < self.base < s.wheel_speed_max:
            raise ValueError('Base speed must lie inside configured wheel range')
        if not 0 <= self.gain < s.line_gain_max or not 0 < self.duration <= s.line_max_duration_s:
            raise ValueError('Invalid line gain or duration')
        if self.mode == 'line' and not self.sensor.calibrated:
            raise ValueError('Calibrate before line following')
        if self.mode not in ('line', 'straight'):
            raise ValueError('Unknown paired-wheel mode')
        self.limit = min(s.correction_max, self.base, s.wheel_speed_max - self.base)
        r.fault, r.running = '', False
        r.ready[:] = [False, False]
        r.stopped[:] = [True, True]
        r.target[:] = [0.0, 0.0]
        r.velocity[:] = [0.0, 0.0]
        r.voltage[:] = [0.0, 0.0]
        self.sensor.max_read_us = 0
        self.data.metadata = {
            'loop_type': self.mode, 'control_structure': 'outer P / inner PI-FF',
            'base_rad_per_s': self.base, 'outer_gain_rad_per_s_per_mm': self.gain,
            'correction_limit_rad_per_s': self.limit,
            'wheel_min_rad_per_s': 0, 'wheel_max_rad_per_s': s.wheel_speed_max,
            'velocity_unit': 'rad/s',
            'duration_s': self.duration, 'motor_period_ms': s.motor_period_ms,
            'motor_priority': s.motor_priority, 'line_period_ms': s.line_period_ms,
            'line_priority': s.line_priority, 'ui_period_ms': s.ui_period_ms,
            'ui_priority': s.ui_priority, 'log_period_ms': s.line_log_period_ms,
            'battery_voltage_V': s.battery_voltage, 'voltage_min_V': s.voltage_min,
            'voltage_max_V': s.voltage_max, 'adc_method': 'read_timed_multi',
            'adc_hz': cfg.ADC_HZ, 'adc_samples_per_channel': cfg.ADC_SAMPLES,
            'adc_timer': cfg.ADC_TIMER, 'channels': list(cfg.CHANNELS),
            'adc_pins': list(cfg.SENSOR_PINS), 'emitters': 'both_full_brightness',
            'positions_mm': list(self.sensor.positions), 'min_span_ADC': cfg.MIN_SPAN,
            'calibration_file': cfg.CALIBRATION_FILE,
            'centroid_convention': 'positive left, zero centered; nan undefined',
            'telemetry_timing': 'latest motor snapshots, not simultaneous ADC samples',
            'feedforward_model': 'K_FF * wheel_setpoint',
            'kp_kff_unit': 'V/(rad/s)', 'ki_unit': 'V/rad',
            'preparation_min_ms': s.preparation_min_ms,
            'rest_velocity_limit_rad_per_s': s.rest_velocity_limit,
            'rest_samples_required': s.rest_samples_required,
            'line_stale_ms': s.line_stale_ms,
            'encoder_counts_per_revolution': cfg.ENCODER_COUNTS_PER_REVOLUTION,
        }
        for side, gains in self.gain_banks.items():
            for key in ('K_P', 'K_I', 'K_FF'):
                self.data.metadata[side + '_' + key] = getattr(gains, key).value
        self.prep_start = ticks_us()
        r.active = True
        self.state = S1_PREP

    def _sample(self):
        r = self.run_request
        centroid = self.sensor.read() if self.mode == 'line' else None
        weight = self.sensor.total_weight if self.mode == 'line' else 0.0
        now = ticks_us()
        elapsed = ticks_diff(now, self.start_us) / 1000
        if self.mode == 'line' and centroid is None:
            self.data.record(elapsed, None, weight, r)
            self._stop('failed', 'Undefined centroid: total normalized weight is zero')
            return
        correction = (min(self.limit, max(-self.limit, self.gain * centroid))
                      if self.mode == 'line' else 0.0)
        r.target[0], r.target[1] = self.base - correction, self.base + correction
        r.updated_us = now
        r.running = True
        if elapsed >= self.next_log_ms:
            self.data.record(elapsed, centroid, weight, r)
            self.next_log_ms = elapsed + self.settings.line_log_period_ms

    def _stop(self, status, reason=''):
        r = self.run_request
        r.stop, r.running = True, False
        r.target[:] = [0.0, 0.0]
        self.data.status, self.data.reason = status, reason
        self.data.metadata.setdefault("loop_type", r.request or "line")
        if self.started:
            self.data.metadata['actual_duration_ms'] = ticks_diff(ticks_us(), self.start_us) / 1000
        self.state = S3_STOP

    def run(self):
        while True:
            r = self.run_request
            try:
                if self.state == S0_IDLE:
                    if r.request is not None and not self.print_flag.print_flag:
                        self._prepare()
                elif self.state == S1_PREP:
                    if r.fault:
                        self._stop('failed', r.fault)
                    elif r.stop:
                        self._stop('stopped', 'User stop')
                    elif all(r.ready):
                        self.start_us = ticks_us()
                        self.next_log_ms = 0.0
                        self.started = True
                        self.data.status = 'running'
                        self.state = S2_FOLLOW
                        self._sample()
                    elif ticks_diff(ticks_us(), self.prep_start) > (self.settings.preparation_timeout_ms + 100) * 1000:
                        self._stop('failed', 'Wheel preparation timeout')
                elif self.state == S2_FOLLOW:
                    if r.fault:
                        self._stop('failed', r.fault)
                    elif r.stop:
                        self._stop('stopped', 'User stop')
                    elif ticks_diff(ticks_us(), self.start_us) >= self.duration * 1000000:
                        self._stop('complete')
                    else:
                        self._sample()
                elif self.state == S3_STOP:
                    # Only announce data after BOTH motor owners acknowledge disable.
                    if all(r.stopped):
                        r.active, r.request = False, None
                        self.data.metadata['expected_samples'] = self.data.count
                        self.data.metadata['adc_max_read_us'] = self.sensor.max_read_us
                        self.print_flag.print_flag = True
                        self.state = S0_IDLE
                else:
                    raise RuntimeError('Invalid TaskLine state')
            except (ValueError, TypeError, RuntimeError, OSError) as error:
                self._stop('failed', str(error))
            yield self.state

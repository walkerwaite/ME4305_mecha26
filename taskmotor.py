"""One motor/encoder owner: finite OL/PI-FF tests and continuous wheel tracking."""
from time import ticks_us, ticks_diff
from math import isfinite

S0_INIT = 0
S1_PREP = 1
S2_COLLECT = 2
S3_TRACK = 3


class TaskMotor:
    def __init__(self, encoder, motor, duty_cycle, data, flag, print_flag,
                 gains, v_sp, loop, controller, settings, motor_name, line_run):
        self.encoder, self.motor = encoder, motor
        self.duty_cycle, self.data = duty_cycle, data
        self.run_flag, self.print_flag = flag, print_flag
        self.gains, self.v_sp, self.loop_type = gains, v_sp, loop
        self.controller, self.settings = controller, settings
        self.motor_name, self.line = motor_name, line_run
        self.side = 0 if motor_name == 'left' else 1
        self.state = S0_INIT
        self.line_mode = False

    def _prepare(self, line_mode=False):
        self.line_mode = line_mode
        self.motor.disable()
        self.controller.reset()
        self.data.reset()
        self.test_mode = 'closed' if line_mode else self.loop_type.value
        s = self.settings
        self.test_battery = float(s.battery_voltage)
        if not isfinite(self.test_battery) or self.test_battery <= 0:
            raise ValueError('Battery reference must be positive')
        if max(abs(self.controller.V_min), abs(self.controller.V_max)) > self.test_battery:
            raise ValueError('Controller voltage limits exceed battery reference')
        self.test_samples = s.samples_after_initial + 1
        self.test_kp = self.test_ki = self.test_kff = 0.0
        self.test_setpoint, self.test_duty = 0.0, None
        if self.test_mode == 'closed':
            self.test_kp = float(self.gains.K_P.value)
            self.test_ki = float(self.gains.K_I.value)
            self.test_kff = float(self.gains.K_FF.value)
            self.test_setpoint = 0.0 if line_mode else float(self.v_sp.value)
            for value, bound in ((self.test_kp, s.kp_max),
                                 (self.test_ki, s.ki_max), (self.test_kff, s.kff_max)):
                if not isfinite(value) or not 0 <= value < bound:
                    raise ValueError('Invalid PI-FF gain')
            if self.test_kp == 0 and self.test_ki == 0:
                raise ValueError('At least one feedback gain must be positive')
            if not isfinite(self.test_setpoint) or abs(self.test_setpoint) >= s.setpoint_max:
                raise ValueError('Invalid velocity setpoint')
        elif self.test_mode == 'open':
            self.test_duty = float(self.duty_cycle.value)
            if not isfinite(self.test_duty) or not -100 < self.test_duty < 100:
                raise ValueError('Invalid duty cycle')
        else:
            raise ValueError('Unknown control mode')
        if not line_mode:
            self.data.metadata = {
                'motor': self.motor_name, 'loop_type': self.test_mode,
                'control_structure': 'PI-FF' if self.test_mode == 'closed' else 'open_loop',
                'proportional_gain': self.test_kp, 'integral_gain': self.test_ki,
                'feedforward_gain': self.test_kff,
                'feedforward_model': 'K_FF * setpoint',
                'integral_gain_unit': 'V/count', 'gain_unit': 'V/(counts/s)',
                'anti_windup': 'conditional integration and integral voltage clamp',
                'setpoint_counts_per_s': self.test_setpoint,
                'open_loop_duty_percent': self.test_duty,
                'motor_period_ms': s.motor_period_ms, 'ui_period_ms': s.ui_period_ms,
                'motor_priority': s.motor_priority, 'ui_priority': s.ui_priority,
                'duration_ms': (self.test_samples - 1) * s.motor_period_ms,
                'expected_samples': self.test_samples, 'battery_voltage_V': self.test_battery,
                'voltage_min_V': self.controller.V_min if self.test_mode == 'closed' else -self.test_battery,
                'voltage_max_V': self.controller.V_max if self.test_mode == 'closed' else self.test_battery,
                'time_unit': 'ms', 'position_unit': 'encoder_counts',
                'velocity_unit': 'counts/s', 'voltage_unit': 'V', 'step_time_ms': 0,
                'preparation_min_ms': s.preparation_min_ms,
                'rest_velocity_limit_counts_per_s': s.rest_velocity_limit,
                'rest_samples_required': s.rest_samples_required,
            }
        self.preparation_start = ticks_us()
        self.rest_count = 0
        self.encoder.zero()
        self.state = S1_PREP

    def _voltage_for(self, velocity, dt):
        if self.test_mode == 'closed':
            return self.controller.run(velocity, self.test_kp, self.test_setpoint,
                                       self.test_ki, self.test_kff, dt)
        return self.test_duty * self.test_battery / 100

    def _start_response(self):
        self.encoder.zero()
        self.controller.reset()
        self.time_trial_start = ticks_us()
        self.motor.enable()
        if self.line_mode:
            self.line.ready[self.side] = True
            self.state = S3_TRACK
            return
        voltage = self._voltage_for(0.0, 0.0)
        self.motor.set_effort(100 * voltage / self.test_battery)
        self.data.record(0, 0, self.test_setpoint, 0.0, voltage)
        self.data.status = 'running'
        self.state = S2_COLLECT

    def _finish(self, status='complete', reason=''):
        self.motor.disable()
        self.controller.reset()
        self.state = S0_INIT
        if self.line_mode:
            self.line.stopped[self.side] = True
            self.line.ready[self.side] = False
            self.line.voltage[self.side] = 0.0
            if status == 'failed':
                self.line.fault = self.motor_name + ': ' + reason
                self.line.stop = True
            self.line_mode = False
            return
        self.data.status, self.data.reason = status, reason
        if status == 'complete':
            self.data.metadata['actual_duration_ms'] = ticks_diff(
                ticks_us(), self.time_trial_start) / 1000
        self.run_flag.run_flag = False
        self.run_flag.stop_requested = False
        self.print_flag.print_flag = True

    def run(self):
        while True:
            try:
                if self.state == S0_INIT:
                    self.motor.disable()
                    if self.line.active and not self.line.stop:
                        self.line.stopped[self.side] = False
                        self._prepare(True)
                    elif self.run_flag.run_flag and not self.print_flag.print_flag:
                        if self.run_flag.stop_requested:
                            self.line_mode = False
                            self.data.reset()
                            self._finish('aborted', 'Stopped before preparation')
                        else:
                            self._prepare(False)
                elif self.state == S1_PREP:
                    stopped = (self.line.stop if self.line_mode else
                               (not self.run_flag.run_flag or self.run_flag.stop_requested))
                    if stopped:
                        self._finish('aborted', 'Stop requested')
                    else:
                        self.encoder.update()
                        if abs(self.encoder.get_velocity()) <= self.settings.rest_velocity_limit:
                            self.rest_count += 1
                        else:
                            self.rest_count = 0
                        elapsed = ticks_diff(ticks_us(), self.preparation_start) / 1000
                        if (elapsed >= self.settings.preparation_min_ms
                                and self.rest_count >= self.settings.rest_samples_required):
                            self._start_response()
                        elif elapsed >= self.settings.preparation_timeout_ms:
                            self._finish('failed', 'Motor did not settle before timeout')
                elif self.state == S2_COLLECT:
                    if not self.run_flag.run_flag or self.run_flag.stop_requested:
                        self._finish('aborted', 'Stop requested')
                    else:
                        self.encoder.update()
                        elapsed = ticks_diff(ticks_us(), self.time_trial_start) / 1000
                        velocity = self.encoder.get_velocity()
                        voltage = self._voltage_for(velocity, self.encoder.dt)
                        self.motor.set_effort(100 * voltage / self.test_battery)
                        self.data.record(elapsed, self.encoder.get_position(),
                                         self.test_setpoint, velocity, voltage)
                        if self.data.count >= self.test_samples:
                            self._finish()
                elif self.state == S3_TRACK:
                    if self.line.stop or not self.line.active:
                        self._finish('stopped')
                    elif not self.line.running:
                        self.encoder.update()
                    else:
                        if ticks_diff(ticks_us(), self.line.updated_us) > self.settings.line_stale_ms * 1000:
                            raise RuntimeError('Stale wheel request')
                        self.encoder.update()
                        self.test_setpoint = self.line.target[self.side]
                        velocity = self.encoder.get_velocity()
                        voltage = self._voltage_for(velocity, self.encoder.dt)
                        self.motor.set_effort(100 * voltage / self.test_battery)
                        self.line.velocity[self.side] = velocity
                        self.line.voltage[self.side] = voltage
                else:
                    raise RuntimeError('Invalid TaskMotor state')
            except (ValueError, TypeError, RuntimeError, OSError) as error:
                if self.line_mode:
                    self._finish('failed', str(error))
                else:
                    # A finite test fault must stop the entire application, too.
                    self.motor.disable()
                    raise
            yield self.state

"""Lab 0x08. Contains combined TaskLine and TaskIMU into one MOTION_TASK.

Startup: motors off -> calibration/reuse/motor-only -> scheduler.
Right encoder: TIM2 CH1 PA15, CH2 PB3 (moved away from ADC PA0/PA1).
Left encoder: TIM1 CH1 PA8, CH2 PA9.
Right motor: TIM3 CH1 PB4, direction PB10, sleep PC8.
Left motor: TIM3 CH2 PB5, direction PC7, sleep PC6.
PA0/PA1/PA6/PA7 are now sensor ADC pins, never motor/encoder pins.
Sensor supply must follow the lab's AVDD (3.3 V) requirement, common ground.
"""
from pyb import Pin, Timer

# Disable both motor drivers immediately.
Pin(Pin.cpu.C8, Pin.OUT_PP, value=0)  # Right motor sleep
Pin(Pin.cpu.C6, Pin.OUT_PP, value=0)  # Left motor sleep

import gc
import cotask
import sys

gc.collect()
from taskuser import TaskUser
gc.collect()
from taskmotor import TaskMotor
gc.collect()
from taskmotion import TaskMotion
gc.collect()
from encoder import Encoder
gc.collect()
from motordriver import MotorDriver
gc.collect()
from closedloop import ClosedLoop
gc.collect()
from linesensor import LineSensor
gc.collect()
from romi_config import Settings
gc.collect()
from array import array

# Lab 0x07 imports
from pyb import I2C
from imudriver import BNO055
from imu_calibration import startup_calibration

# Sleep functionality
from time import sleep_ms


class Value:
    def __init__(self, value=None):
        self.value = value

class RunFlag:
    def __init__(self):
        self.run_flag = False
        self.stop_requested = False

class PrintFlag:
    def __init__(self):
        self.print_flag = False

class ExitFlag:
    def __init__(self):
        self.exit_flag = False

class Gains:
    def __init__(self):
        self.K_P = Value(0.001)
        self.K_I = Value(0.01)
        self.K_FF = Value(0.002)

class MotorData:
    def __init__(self, capacity):
        self.time = array('f', [0] * capacity)
        self.pos = array('i', [0] * capacity)
        self.setpoint = array('f', [0] * capacity)
        self.vel = array('f', [0] * capacity)
        self.voltage = array('f', [0] * capacity)
        self.reset()

    def reset(self):
        self.count = 0
        self.metadata = {}
        self.status, self.reason = "idle", ""

    def record(self, time, pos, setpoint, vel, voltage):
        i = self.count
        if i >= len(self.time):
            raise ValueError("Motor log is full")
        self.time[i], self.pos[i], self.setpoint[i] = time, pos, setpoint
        self.vel[i], self.voltage[i] = vel, voltage
        self.count += 1

    def release(self):
        self.time = self.pos = self.setpoint = self.vel = self.voltage = None

class LineData:
    def __init__(self, capacity):
        self.capacity = capacity
        self.columns = ()
        self.reset()

    def reset(self):
        if not self.columns:
            self.columns = tuple(
                array('f', [0] * self.capacity) for _ in range(9)
            )

        self.count, self.metadata = 0, {}
        self.status, self.reason = "idle", ""

    def record(self, time, centroid, weight, run):
        if self.count >= len(self.columns[0]):
            raise ValueError("Line log is full")
        values = (time, float('nan') if centroid is None else centroid, weight,
                  run.target[0], run.target[1], run.velocity[0], run.velocity[1],
                  run.voltage[0], run.voltage[1])
        for column, value in zip(self.columns, values):
            column[self.count] = value
        self.count += 1

    def release(self):
        self.columns = ()

class LineRun:
    """Motion task for handling line following and outer wheel requests."""
    def __init__(self, duration):
        self.request = None       # None, "line", or "straight" (outer loop off).
        self.active = self.running = self.stop = False
        self.ready = [False, False]
        self.stopped = [True, True]
        self.target = [0.0, 0.0] # left, right, encoder counts/s
        self.velocity = [0.0, 0.0]
        self.voltage = [0.0, 0.0]
        self.updated_us = 0
        self.fault = ""
        self.straight_speed = 400.0
        self.base = Value(1000.0)
        self.gain = Value(30.0)    # (counts/s)/mm; provisional outer P gain.
        self.duration = Value(duration)

class IMUData:
    """Latest measurements published by Motion Task.
        Collects heading [rad], yaw rate [rad/s], and timestamp [us]."""

    def __init__(self):
        self.heading_rad = 0.0
        self.yaw_rate_rad_s = 0.0
        self.updated_us = 0
        self.valid = False
        self.fault = ""



def main():
    # Disable the existing hardware even if later construction fails.
    right_sleep = Pin(Pin.cpu.C8, Pin.OUT_PP, value=0)
    left_sleep = Pin(Pin.cpu.C6, Pin.OUT_PP, value=0)
    motors, logs = [], []
    sensor = ui = task_list = None
    reason, message = 'exit', ''
    try:
        settings = Settings()
        pwm = Timer(3, freq=20000)
        right = MotorDriver(Pin.cpu.B4, Pin.cpu.B10, Pin.cpu.C8, pwm, 1)
        motors.append(right)
        left = MotorDriver(Pin.cpu.B5, Pin.cpu.C7, Pin.cpu.C6, pwm, 2)
        motors.append(left)
        r_encoder = Encoder(2, Pin.cpu.A15, Pin.cpu.B3)
        l_encoder = Encoder(1, Pin.cpu.A8, Pin.cpu.A9)
        sensor = LineSensor()
        duty, velocity, loop = Value(None), Value(None), Value('open')
        exit_flag = ExitFlag()
        left_flag, right_flag = RunFlag(), RunFlag()
        left_print, right_print, line_print = PrintFlag(), PrintFlag(), PrintFlag()
        banks = {'left': Gains(), 'right': Gains()}
        left_data = MotorData(settings.samples_after_initial + 1)
        right_data = MotorData(settings.samples_after_initial + 1)
        # Reserve the maximum finite run once; no append growth during control.
        capacity = int(settings.line_max_duration_s * 1000 / settings.line_log_period_ms) + 3
        line_data = LineData(capacity)
        logs.extend((left_data, right_data, line_data))
        line_run = LineRun(settings.line_duration_s)
        line_run.base.value = settings.maneuver_base_speed
        ui = TaskUser(left_data, left_flag, left_print, right_data, right_flag,
                      right_print, exit_flag, duty, banks, velocity, loop,
                      settings, line_run, line_data, line_print, sensor)

        # Integrate IMU into system, calibrate, and pass to TaskLine for lane-change control.
        # Use the I2C bus number and wiring specified for your lab hardware.
        # pyb.I2C selects the hardware SCL/SDA pins from the bus number.
        
        # Reset the IMU through PC9 before initialization.
        imu_reset = Pin(Pin.cpu.C9, Pin.OUT_OD, value=0)
        sleep_ms(5)
        imu_reset.high()

        # Initialize I2C communication and the IMU driver.
        imu_i2c = I2C(1, I2C.CONTROLLER, baudrate=100_000)
        imu = BNO055(imu_i2c, reset_pin=imu_reset)
        imu_data = IMUData()

        # Startup calibration: load saved offsets or calibrate and save; call BEFORE scheduling.
        startup_calibration(imu)
       
        # Calibration deliberately precedes task creation and profiling.
        ui.calibrate_startup()


        left_task = TaskMotor(l_encoder, left, duty, left_data, left_flag,
                             left_print, banks['left'], velocity, loop,
                             ClosedLoop(settings.voltage_min, settings.voltage_max),
                             settings, 'left', line_run)
        right_task = TaskMotor(r_encoder, right, duty, right_data, right_flag,
                              right_print, banks['right'], velocity, loop,
                              ClosedLoop(settings.voltage_min, settings.voltage_max),
                              settings, 'right', line_run)
        motion_task = TaskMotion(sensor, imu, imu_data, line_run, line_data, line_print, settings, banks)
       
        # imu_task = TaskIMU(imu, imu_data) # Create IMU task
        
        ui.print_startup_menu()
        gc.collect()
        task_list = cotask.TaskList()
        cotask.task_list = task_list
        for owner, name, priority, period in (
                (right_task, 'Right Motor', settings.motor_priority, settings.motor_period_ms),
                (left_task, 'Left Motor', settings.motor_priority, settings.motor_period_ms),
                (motion_task, 'Motion', settings.motion_priority, settings.motion_period_ms),
                (ui, 'User Interf', settings.ui_priority, settings.ui_period_ms)):
                
            task_list.append(cotask.Task(owner.run(), name=name, priority=priority,
                                        period=period, profile=True))
        while not exit_flag.exit_flag:
            task_list.pri_sched()
    except KeyboardInterrupt:
        reason = 'interrupted'
    except Exception as error:
            right_sleep.low()
            left_sleep.low()

            sys.print_exception(error)
            reason, message = "failed", str(error)
    finally:
        right_sleep.low()
        left_sleep.low()
        for motor in motors:
            motor.disable()
        if sensor is not None:
            sensor.close()

            # Release sensor acquisition buffers and arrays.
            sensor.buffers = ()
            sensor.adcs = ()
            sensor.raw = None
            sensor.weights = None
            sensor.white = None
            sensor.span = None
            sensor.positions = None
        if ui is not None:
            ui.report_shutdown(reason, message)
        elif message:
            print('Startup failed: ' + message)
        
        # Release recorded data after transmission and 
        # after scheduling has stopped.
        for log in logs:
            log.release()
            log.metadata.clear()
        # Release UI display strings that are no longer needed.
        if ui is not None:
            ui.menu_lines = ()
            ui.gain_menu_lines = ()
            ui.line_menu_lines = ()
            ui.header_lines = ()
            ui.pending_commands.clear()
            ui.char_buf.clear()

            # CHECK UI TASK
            print("Keyboard checks during motion:", ui.motion_input_checks)
        
        if task_list is not None:
            try:
                gc.collect()
                print("\nTask Profile:")
                print(task_list.profile())
            except MemoryError:
                print("Task profile unavailable: insufficient MCU memory")        


if __name__ == '__main__':
    main()
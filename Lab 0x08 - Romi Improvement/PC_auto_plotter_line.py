"""PC-only finite line/straight experiment; reuses the Lab 0x05 serial helpers.

First complete startup calibration in PuTTY, then close PuTTY. Configure these
values BEFORE running. Requires PC_auto_plotter_CL.py beside this file, pyserial,
and matplotlib. No separate automated firmware. e is sent after collection.
"""
import os
from datetime import datetime
from math import isfinite, isclose
from matplotlib import pyplot
from serial import Serial, SerialException
import PC_auto_plotter_CL as pc

SERIAL_PORT = 'COM8'
MODE = 'line'               # 'straight' is an equal-speed, outer-loop-off test.
BASE_SPEED = 1000.0          # counts/s; use a speed verified under load.
OUTER_K_P = 30.0             # (counts/s)/mm; starting value, tune on your robot.
DURATION_S = 5.0           # finite experiment; maximum is firmware setting (60 s).
LEFT_GAINS = (0.001, 0.01, 0.002)   # K_P, K_I, K_FF: replace with loaded tuning.
RIGHT_GAINS = (0.001, 0.01, 0.002)
HEADERS = ('time_ms', 'centroid_mm', 'total_weight',
           'left_setpoint_counts_per_s', 'right_setpoint_counts_per_s',
           'left_velocity_counts_per_s', 'right_velocity_counts_per_s',
           'left_voltage_V', 'right_voltage_V')


def request_run(ser):
    pc.send_command(ser, 'h')
    pc.wait_for_text(ser, '>: ')
    pc.send_command(ser, 'k')
    pc.wait_for_text(ser, 'Select p/i/f for gains or v for setpoint velocity:')
    for target, values in (('left', LEFT_GAINS), ('right', RIGHT_GAINS)):
        pc.configure_gains(ser, *values, target=target)
    pc.send_command(ser, 'h')
    pc.wait_for_text(ser, '>: ')
    pc.send_command(ser, 'n')
    pc.wait_for_text(ser, 'Select b for base, p for outer gain, t for duration, g to go:')
    for command, prompt, ack, value in (
            ('b', 'Enter base velocity [counts/s]: ', 'base set to ', BASE_SPEED),
            ('p', 'Enter outer gain [(counts/s)/mm]: ', 'outer_gain set to ', OUTER_K_P),
            ('t', 'Enter run duration [s]: ', 'duration set to ', DURATION_S)):
        pc.send_command(ser, command)
        pc.wait_for_text(ser, prompt)
        pc.send_number(ser, value, ack)
    if MODE == 'line':
        pc.send_command(ser, 'g')
    else:
        pc.send_command(ser, 'h')
        pc.wait_for_text(ser, '>: ')
        pc.send_command(ser, 'k')
        pc.wait_for_text(ser, 'Select p/i/f for gains or v for setpoint velocity:')
        pc.send_command(ser, 'v')
        pc.wait_for_text(ser, 'Enter setpoint velocity [counts/s]: ')
        pc.send_number(ser, BASE_SPEED, 'Velocity set to ')
        pc.send_command(ser, 'b')


def validate_run(metadata, columns):
    if metadata.get('loop_type') != MODE:
        raise ValueError('Firmware returned the wrong experiment mode')
    # Failed preparation can legitimately produce an empty diagnostic dataset.
    if metadata.get('status') == 'failed' and not columns[0]:
        return
    for key, expected in (('base_counts_per_s', BASE_SPEED),
                          ('outer_gain_counts_per_s_per_mm', OUTER_K_P),
                          ('duration_s', DURATION_S)):
        if not isclose(float(metadata[key]), expected, rel_tol=1e-5, abs_tol=1e-6):
            raise ValueError('Configuration mismatch: ' + key)
    for target, gains in (('left', LEFT_GAINS), ('right', RIGHT_GAINS)):
        for key, expected in zip(('K_P', 'K_I', 'K_FF'), gains):
            if not isclose(float(metadata[target + '_' + key]), expected, rel_tol=1e-5, abs_tol=1e-8):
                raise ValueError('Wheel gain mismatch: ' + target + '_' + key)
    for c in columns[1]:
        if isfinite(c) and abs(c) > 24.001:
            raise ValueError('Centroid outside sensor span')
    for column in columns[3:5]:
        if any(v < 0 or v > float(metadata['wheel_max_counts_per_s']) + 1e-4 for v in column):
            raise ValueError('Wheel request outside recorded limits')
    for column in columns[7:9]:
        if any(v < float(metadata['voltage_min_V']) - 1e-4
               or v > float(metadata['voltage_max_V']) + 1e-4 for v in column):
            raise ValueError('Wheel voltage outside recorded limits')


def save_plot(filename, metadata, columns):
    figure, axes = pyplot.subplots(3, 1, figsize=(10, 9), sharex=True)
    try:
        seconds = [t / 1000 for t in columns[0]]
        # NaN samples produce gaps and are not drawn as valid centroids.
        axes[0].plot(seconds, columns[1], color='black', label='Measured centroid')
        axes[0].axhline(0, color='gray', linestyle='--', label='Center reference')
        axes[0].set_ylabel('Centroid [mm]\npositive left')
        for column, label, style in ((5, 'Left measured', '-'), (6, 'Right measured', '-'),
                                     (3, 'Left requested', '--'), (4, 'Right requested', '--')):
            axes[1].plot(seconds, columns[column], style, label=label)
        axes[1].set_ylabel('Wheel velocity [counts/s]')
        axes[2].plot(seconds, columns[7], label='Left voltage')
        axes[2].plot(seconds, columns[8], label='Right voltage')
        axes[2].set_ylabel('Voltage command [V]')
        axes[2].set_xlabel('MCU elapsed time [s]')
        for axis in axes:
            axis.legend(loc='best', fontsize=8)
            axis.grid(True, alpha=0.3)
        figure.suptitle('{}; status={}\nBase={} counts/s, outer Kp={} (counts/s)/mm\n'
                       'Left PI-FF={}, right PI-FF={}; inner/outer periods={}/{} ms'.format(
                           MODE, metadata.get('status'), BASE_SPEED, OUTER_K_P,
                           LEFT_GAINS, RIGHT_GAINS, metadata.get('motor_period_ms'),
                           metadata.get('line_period_ms')), fontsize=10)
        figure.tight_layout(rect=(0, 0, 1, 0.91))
        with open(filename, 'xb') as file:
            figure.savefig(file, format='png', dpi=200)
    finally:
        pyplot.close(figure)


def main():
    if MODE not in ('line', 'straight'):
        raise ValueError('MODE must be line or straight')
    if not (0 < BASE_SPEED < 3000 and 0 <= OUTER_K_P < 100 and 0 < DURATION_S <= 30):
        raise ValueError('Requested settings exceed default firmware bounds')
    pc.validate_gains(*LEFT_GAINS)
    pc.validate_gains(*RIGHT_GAINS)
    pc.FIRST_RESPONSE_TIMEOUT = DURATION_S + 20.0
    os.makedirs(pc.OUTPUT_DIRECTORY, exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')
    name = os.path.join(pc.OUTPUT_DIRECTORY, '{}_{}_base{}_outer{}'.format(
        timestamp, MODE, pc.filename_number_label(BASE_SPEED), pc.filename_number_label(OUTER_K_P)))
    try:
        print('Opening {} at {} baud'.format(SERIAL_PORT, pc.BAUDRATE))
        with Serial(SERIAL_PORT, baudrate=pc.BAUDRATE, timeout=pc.SERIAL_TIMEOUT) as ser:
            try:
                ser.reset_input_buffer()
                request_run(ser)
                metadata, description, headers, columns = pc.collect_dataset(ser, HEADERS, paired=True)
                validate_run(metadata, columns)
                pc.save_csv(name + '.csv', description, headers, columns)
                save_plot(name + '.png', metadata, columns)
                print('Saved {} samples; status={}: {}'.format(len(columns[0]), metadata.get('status'), name))
                if metadata.get('reason'):
                    print('Run note: ' + metadata['reason'])
                pc.terminate(ser, os.path.join(pc.OUTPUT_DIRECTORY, timestamp + '_task_profile.txt'))
            except pc.FirmwareStoppedError:
                raise
            except (KeyboardInterrupt, Exception):
                pc.send_safe_exit(ser)
                raise
    except KeyboardInterrupt:
        print('Interrupted; exit requested and serial port closed.')
    except (SerialException, TimeoutError, ValueError, OSError, KeyError, pc.FirmwareStoppedError) as error:
        raise SystemExit('Automated collection failed: ' + str(error)) from error


if __name__ == '__main__':
    main()

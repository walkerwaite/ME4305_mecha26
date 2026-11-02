"""Run configured PI-FF wheel tests, save their data, and plot responses.

Close PuTTY and start the revised Romi firmware before running this script.
Dependencies: pyserial and matplotlib. Output files go to the results folder beside this script.
"""

import os
from datetime import datetime
from decimal import Decimal
from math import isfinite, isclose
from time import monotonic
from matplotlib import pyplot
from serial import Serial, SerialException
from serial.tools import list_ports

# Set this to Romi's current VCP
SERIAL_PORT = "COM8"
BAUDRATE = 115200
SERIAL_TIMEOUT = 0.25
PROMPT_TIMEOUT = 10.0
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0
VERBOSE_SERIAL = False
OUTPUT_DIRECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

# Example starting trials, not a certified stability range. Manually verify the
# baseline and adjust gains based on measured behavior. Keep the other conditions
# fixed; periods, duration, preparation, voltage limits, and battery reference
# remain firmware settings.
# motor, command, K_P, K_I, K_FF, setpoint [counts/s].
# Zero I/FF preserves your P baseline. Enter manually verified loaded gains here.
TESTS = (
    ("left", "l", 0.004, 0.0, 0.0, 400.0),
    ("right", "r", 0.004, 0.0, 0.0, 400.0),
)

LINE_ENDING = "\r\n"
EXPECTED_HEADERS = (
    "time_ms", "position_counts", "setpoint_counts_per_s",
    "velocity_counts_per_s", "voltage_command_V")


class FirmwareStoppedError(RuntimeError):
    """The MCU explicitly reported an aborted/failed test or shutdown."""


def decimal_text(value):
    """Encode decimal numbers without e/E, which the MCU treats as exit."""
    text = format(Decimal(str(value)), "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    if len(text) > 64:
        raise ValueError("Numeric entry exceeds the MCU's 64-character buffer")
    return text or "0"


def resolve_serial_port():
    if SERIAL_PORT:
        return SERIAL_PORT
    ports = list(list_ports.comports())
    usb_ports = [port for port in ports if port.vid is not None]
    candidates = usb_ports if usb_ports else ports
    if len(candidates) == 1:
        return candidates[0].device
    available = ", ".join(
        "{} ({})".format(port.device, port.description) for port in ports)
    raise ValueError("Set SERIAL_PORT to Romi's VCP. Available ports: "
                     + (available or "none"))


def validate_tests():
    if not TESTS:
        raise ValueError("Configure at least one response")
    for motor, command, kp, ki, kff, setpoint in TESTS:
        if (motor, command) not in (("left", "l"), ("right", "r")):
            raise ValueError("Motor names and commands must match")
        validate_gains(kp, ki, kff)
        if not isfinite(setpoint) or not -10000 < setpoint < 10000:
            raise ValueError("Invalid setpoint")
        decimal_text(setpoint)


def validate_gains(kp, ki, kff):
    for value, bound in ((kp, 0.1), (ki, 1.0), (kff, 0.1)):
        if not isfinite(value) or not 0 <= value < bound:
            raise ValueError("Gain outside firmware entry range")
        decimal_text(value)
    if kp == 0 and ki == 0:
        raise ValueError("At least one feedback gain must be positive")


def check_firmware_status(text):
    if ("Test aborted" in text or "Test failed" in text
            or "Exiting Program" in text):
        raise FirmwareStoppedError(text.strip())
    if "Invalid entry:" in text or "Value not changed" in text:
        raise ValueError("Firmware rejected the configuration: " + text.strip())


def wait_for_text(ser, expected, timeout=PROMPT_TIMEOUT):
    """Read bounded chunks until a prompt, including prompts without newlines."""
    deadline = monotonic() + timeout
    text = ""
    while monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        text += byte.decode("utf-8", errors="replace")
        check_firmware_status(text)
        if expected in text:
            if VERBOSE_SERIAL:
                print("Serial message: " + text.strip())
            return text
        if len(text) > 8192:
            text = text[-4096:]

    # Timeout Error checks
    raise TimeoutError(
        "Did not receive prompt: {!r}\n"
        "Received while waiting: {!r}".format(expected, text))


def read_line_until(ser, deadline):
    """Accumulate a full line across short serial read timeouts."""
    buffer = bytearray()
    while monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        buffer.extend(byte)
        if byte == b"\n":
            return buffer.decode("utf-8", errors="replace").rstrip("\r\n")
        if len(buffer) > 4096:
            raise ValueError("Received an unexpectedly long serial line")
    raise TimeoutError("Timed out waiting for a complete serial line")


def send_command(ser, command):
    # One byte, no line ending.
    ser.write(command.encode("ascii"))
    ser.flush()


def send_number(ser, value, acknowledgement):
    ser.write((decimal_text(value) + LINE_ENDING).encode("ascii"))
    ser.flush()
    wait_for_text(ser, acknowledgement)
    reply = read_line_until(ser, monotonic() + PROMPT_TIMEOUT)
    try:
        confirmed = float(reply.strip())
    except ValueError as error:
        raise ValueError("Invalid numeric acknowledgement: " + reply) from error
    if not isfinite(confirmed) or not isclose(
            confirmed, value, rel_tol=1e-5, abs_tol=1e-8):
        raise ValueError("Acknowledged value differs from requested value")
    wait_for_text(ser, ">: ")
    return confirmed


def configure_gains(ser, kp, ki, kff, target="both"):
    """Call inside the gain menu; update and acknowledge the chosen wheel bank."""
    validate_gains(kp, ki, kff)
    send_command(ser, {"both": "a", "left": "1", "right": "2"}[target])
    wait_for_text(ser, "Gain target: " + target)
    wait_for_text(ser, ">: ")
    values = []
    for command, prompt, ack, value in (
            ("p", "Enter proportional gain Kp: ", "KP set to ", kp),
            ("i", "Enter integral gain Ki: ", "KI set to ", ki),
            ("f", "Enter feedforward gain Kff: ", "KFF set to ", kff)):
        send_command(ser, command)
        wait_for_text(ser, prompt)
        values.append(send_number(ser, value, ack))
    return tuple(values)


def send_CL_test_request(ser, command, kp, v_sp, ki=0.0, kff=0.0):
    send_command(ser, "k")
    wait_for_text(ser, "Select p/i/f for gains or v for setpoint velocity:")
    confirmed = configure_gains(ser, kp, ki, kff,
                                "left" if command == "l" else "right")
    send_command(ser, "v")
    wait_for_text(ser, "Enter setpoint velocity [counts/s]: ")
    confirmed_setpoint = send_number(ser, v_sp, "Velocity set to ")
    send_command(ser, command)
    return confirmed + (confirmed_setpoint,)


def collect_dataset(ser, expected_headers=EXPECTED_HEADERS, paired=False):
    """Return the actual MCU configuration, raw header, and all five columns."""
    metadata, descriptive_header = {}, []
    columns = [[] for _ in expected_headers]
    headers = None
    header_started = header_finished = False
    started = last_line = monotonic()

    while True:
        deadline = (started + FIRST_RESPONSE_TIMEOUT if headers is None
                    else last_line + BETWEEN_LINES_TIMEOUT)
        line = read_line_until(ser, deadline).strip()
        last_line = monotonic()
        check_firmware_status(line)
        if not line:
            continue

        if line == "# BEGIN TEST":
            if header_started or headers is not None:
                raise ValueError("Unexpected second dataset header")
            header_started = True
            descriptive_header.append(line)
            continue

        if line.startswith("#"):
            if not header_started or headers is not None:
                raise ValueError("Metadata is out of order")
            descriptive_header.append(line)
            if line == "# END HEADER":
                header_finished = True
            elif ":" in line:
                key, value = line[1:].split(":", 1)
                metadata[key.strip()] = value.strip()
            if VERBOSE_SERIAL:
                print("Serial message: " + line)
            continue

        if line == "End of data":
            if headers is None:
                raise ValueError("Completion marker arrived before CSV columns")
            break

        if headers is None:
            fields = tuple(field.strip() for field in line.split(","))
            if fields == expected_headers:
                if not header_finished:
                    raise ValueError("Descriptive header was not completed")
                headers = fields
            elif VERBOSE_SERIAL:
                print("Serial message: " + line)
            continue

        fields = line.split(",")
        if len(fields) != len(headers):
            raise ValueError("Malformed response row: " + line)
        try:
            values = [float(field.strip()) for field in fields]
        except ValueError as error:
            raise ValueError("Non-numeric response row: " + line) from error
        if not all(isfinite(value) or (paired and i == 1 and value != value)
                   for i, value in enumerate(values)):
            raise ValueError("Nonfinite response data")
        for column, value in zip(columns, values):
            column.append(value)

    if metadata.get("status") != "complete" and not paired:
        raise ValueError("Dataset does not report successful completion")
    if "expected_samples" not in metadata:
        raise ValueError("Missing configuration field: expected_samples")
    expected_count = int(metadata["expected_samples"])
    if len(columns[0]) != expected_count or (not columns[0] and not paired):
        raise ValueError("Incomplete response: received {} of {} samples".format(
            len(columns[0]), expected_count))
    if any(later <= earlier for earlier, later in zip(columns[0], columns[0][1:])):
        raise ValueError("Sample timestamps are not strictly increasing")

    return metadata, descriptive_header, headers, columns


def validate_configuration(metadata, motor, kp, ki, kff, setpoint, previous):
    if (metadata.get("motor") != motor or metadata.get("control_structure") != "PI-FF"
            or metadata.get("loop_type") != "closed"):
        raise ValueError("MCU reported a different motor or control structure")
    for key, expected in (("proportional_gain", kp),
                          ("integral_gain", ki), ("feedforward_gain", kff),
                          ("setpoint_counts_per_s", setpoint)):
        if not isclose(float(metadata[key]), expected, rel_tol=1e-5, abs_tol=1e-8):
            raise ValueError("MCU configuration differs from the request: " + key)

    # The header documents actual firmware settings; the PC does not set periods.
    fixed_keys = (
        "motor_period_ms", "ui_period_ms", "motor_priority", "ui_priority",
        "duration_ms", "expected_samples", "battery_voltage_V",
        "voltage_min_V", "voltage_max_V", "preparation_min_ms",
        "rest_velocity_limit_counts_per_s", "rest_samples_required",
        "velocity_unit", "gain_unit", "voltage_unit", "step_time_ms")
    for key in fixed_keys:
        if key not in metadata:
            raise ValueError("Missing configuration field: " + key)
        if previous is not None and metadata[key] != previous[key]:
            raise ValueError("Conditions changed within the progression: " + key)


def validate_signals(metadata, columns):
    setpoint = float(metadata["setpoint_counts_per_s"])
    lower, upper = float(metadata["voltage_min_V"]), float(metadata["voltage_max_V"])
    for value in columns[2]:
        if not isclose(value, setpoint, rel_tol=1e-5, abs_tol=1e-6):
            raise ValueError("Logged setpoint changed during the response")
    if any(value < lower - 1e-5 or value > upper + 1e-5 for value in columns[4]):
        raise ValueError("Logged voltage exceeds the stated limits")


def save_csv(filename, descriptive_header, headers, columns):
    """Preserve the MCU header and every recorded column with basic file writes."""
    with open(filename, "x", encoding="utf-8") as file:
        for line in descriptive_header:
            file.write(line + "\n")
        file.write(",".join(headers) + "\n")
        for row in zip(*columns):
            file.write(",".join(format(value, ".12g") for value in row) + "\n")


def save_plot(filename, metadata, columns):
    time_ms, _, setpoint, velocity, voltage = columns
    figure, axes = pyplot.subplots(2, 1, sharex=True, figsize=(9, 7))
    try:
        axes[0].plot(time_ms, velocity, color='black', label="Measured velocity")
        axes[0].plot(time_ms, setpoint, "--", color='black', label="Setpoint")
        axes[0].set_ylabel("Velocity [counts/s]")
        axes[0].legend(loc='lower right')
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(time_ms, voltage, color='black', label="Limited voltage command")
        axes[1].axhline(float(metadata["voltage_max_V"]), color='black',
                        linestyle="--", linewidth=0.8, label="Voltage limits")
        axes[1].axhline(float(metadata["voltage_min_V"]), color='black',
                        linestyle="--", linewidth=0.8)
        axes[1].set_ylabel("Limited voltage command [V]")
        axes[1].set_xlabel("Elapsed time [ms]")
        axes[1].legend(loc='lower right')
        axes[1].grid(True, alpha=0.3)

        figure.suptitle(
            "{} Motor PI-FF Velocity Response\n"
            "Kp={}, Ki={}, Kff={}; setpoint={} counts/s; period={} ms".format(
                metadata["motor"].title(), metadata["proportional_gain"],
                metadata["integral_gain"], metadata["feedforward_gain"],
                metadata["setpoint_counts_per_s"], metadata["motor_period_ms"]),
            fontsize=11)
        figure.tight_layout(rect=(0, 0, 1, 0.94))
        # Opening the binary file exclusively prevents overwriting earlier plots.
        with open(filename, "xb") as file:
            figure.savefig(file, format="png", dpi=200, bbox_inches="tight")
    finally:
        pyplot.close(figure)


def filename_number_label(value):
    return ("neg" if value < 0 else "pos") + decimal_text(abs(value)).replace(".", "p")


def send_safe_exit(ser):
    try:
        send_command(ser, "e")
    except (SerialException, OSError):
        pass

def terminate(ser, filename, timeout=15.0):
    # Request exit without a line ending.
    send_command(ser, "e")

    deadline = monotonic() + timeout
    lines = []
    started = False

    try:
        while True:
            line = read_line_until(ser, deadline)

            if "Task profile unavailable" in line:
                raise ValueError(line)

            # Ignore shutdown messages before the profile.
            if line.strip() == "Task Profile:":
                started = True

            if started:
                lines.append(line)
                # The bottom border marks the complete table.
                if line.lstrip().startswith("└"):
                    break

        # UTF-8 preserves the table borders; "x" prevents overwriting.
        with open(filename, "x", encoding="utf-8") as file:
            file.write("\n".join(lines) + "\n")

    except (KeyboardInterrupt, SerialException,
            TimeoutError, ValueError, OSError) as error:
        print("Firmware exit requested; profile not saved: {}".format(
            str(error) or "capture interrupted"))
        return False

    print("Saved scheduler profile to " + filename)
    return True

def main():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
    previous = None
    os.makedirs(OUTPUT_DIRECTORY, exist_ok=True)
    try:
        validate_tests()
        port = resolve_serial_port()
        print("Opening {} at {} baud".format(port, BAUDRATE))
        with Serial(port, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
            try:
                ser.reset_input_buffer()
                # Verify the running firmware and return to the main menu.
                send_command(ser, "h")
                wait_for_text(ser, ">: ")

                for index, (motor, command, kp, ki, kff, setpoint) in enumerate(TESTS, 1):
                    print("Requesting {} PI-FF response: Kp={}, Ki={}, Kff={}, setpoint={}".format(
                        motor, kp, ki, kff, setpoint))
                    confirmed_kp, confirmed_ki, confirmed_kff, confirmed_setpoint = send_CL_test_request(
                        ser, command, kp, setpoint, ki, kff)
                    metadata, description, headers, columns = collect_dataset(ser)
                    validate_configuration(
                        metadata, motor, confirmed_kp, confirmed_ki, confirmed_kff, confirmed_setpoint, previous)
                    validate_signals(metadata, columns)
                    previous = metadata

                    name = os.path.join(OUTPUT_DIRECTORY, "{}_{}_PIFF_trial{:02d}_kp{}_ki{}_ff{}_sp{}".format(
                        timestamp, motor, index, filename_number_label(kp),
                        filename_number_label(ki), filename_number_label(kff),
                        filename_number_label(setpoint)))
                    data_filename, plot_filename = name + ".csv", name + ".png"
                    save_csv(data_filename, description, headers, columns)
                    save_plot(plot_filename, metadata, columns)
                    print("Saved {} samples to {}".format(len(columns[0]), data_filename))
                    print("Saved plot to " + plot_filename)
                
                # Outside the test loop, before the serial port closes.
                profile_filename = os.path.join(OUTPUT_DIRECTORY, timestamp + "_task_profile.txt")
                terminate(ser, profile_filename)
                
            except FirmwareStoppedError:
                # This marker is emitted only after the MCU has stopped motors.
                raise
            except (KeyboardInterrupt, Exception):
                send_safe_exit(ser)
                raise
    except KeyboardInterrupt:
        print("Collection interrupted; exit requested and serial port closed.")
    except (SerialException, TimeoutError, ValueError, OSError, KeyError,
            FirmwareStoppedError) as error:
        raise SystemExit("Automated collection failed: {}".format(error)) from error


if __name__ == "__main__":
    main()

"""USB VCP owner: cooperative menus, numeric entry, and response transfer."""

from pyb import USB_VCP, delay
from math import isfinite
from line_calibration import load_calibration, save_calibration
import romi_config as cfg

S0_INIT = 0
S1_INPUT = 1
S2_WAIT = 2
S3_PRINT = 3
S4_DIGIT = 4
S5_PARAM = 5
S6_LINE = 6

OPEN_HEADER = "time_ms,position_counts,velocity_counts_per_s"

CLOSED_HEADER = (
    "time_ms,position_counts,setpoint_counts_per_s,"
    "velocity_counts_per_s,voltage_command_V")

LINE_HEADER = ("time_ms,centroid_mm,total_weight,left_setpoint_counts_per_s,"
               "right_setpoint_counts_per_s,left_velocity_counts_per_s,"
               "right_velocity_counts_per_s,left_voltage_V,right_voltage_V")


class TaskUser:
    def __init__(self, left_data, left_flag, left_print, right_data,
                 right_flag, right_print, exit_flag, duty_cycle,
                 gain_banks, V_sp, loop, settings, line_run, line_data, line_print, sensor):
        self.left_data, self.right_data = left_data, right_data
        self.left_flag, self.right_flag = left_flag, right_flag
        self.left_print, self.right_print = left_print, right_print
        self.exit_flag, self.duty_cycle = exit_flag, duty_cycle
        self.gain_banks = gain_banks

        self.motion_input_checks = 0 # Checks for ui issue
        
        self.gain_target = "both"
        self.V_sp, self.loop_type = V_sp, loop
        self.line, self.line_data, self.line_print = line_run, line_data, line_print
        self.sensor = sensor
        self.paired_output = False
        self.line_menu_index = 0
        self.line_menu_waiting = False
        self.settings = settings
        self.vcp = USB_VCP()
        self.state = S0_INIT
        self.data = self.flag = self.print = None
        self.menu_index = self.gain_menu_index = self.row_index = 0
        self.gain_menu_waiting = False
        self.pending_commands = []
        self.char_buf = []
        self.input_target = None
        self.entry_overflow = False
        self.term = {"\r", "\n"}
        self.header_lines = ()
        self.header_index = 0
        self.header_started = False
        self.csv_printed = False

        self.menu_lines = self._menu(
            "ME 4305 Romi Tuning Interface Help Menu", (
                ("h/H", "Print help menu"),
                ("d/D", "Enter duty cycle for the next OPEN-loop test"),
                ("l/L", "Run OPEN-loop response on left motor"),
                ("r/R", "Run OPEN-loop response on right motor"),
                ("k/K", "Configure CLOSED-loop PI-FF control"),
                ("n/N", "Configure line following"),
                ("s/S", "Stop motion; remain in program"),
                ("e/E", "Exit program safely")))
        self.gain_menu_lines = self._menu(
            "ME 4305 Romi Tuning Interface Gain Menu", (
                ("h/H", "Return to the OPEN-loop help menu"),
                ("a/A", "Apply subsequent gains to BOTH wheels (default)"),
                ("1", "Apply subsequent gains to LEFT wheel"),
                ("2", "Apply subsequent gains to RIGHT wheel"),
                ("p/P", "Enter proportional gain K_P [V/(counts/s)]"),
                ("i/I", "Enter integral gain K_I [V/count]"),
                ("f/F", "Enter feedforward gain K_FF [V/(counts/s)]"),
                ("v/V", "Enter signed velocity setpoint [counts/s]"),
                ("l/L", "Run CLOSED-loop PI-FF response on left motor"),
                ("r/R", "Run CLOSED-loop PI-FF response on right motor"),
                ("b/B", "Run BOTH wheels at entered velocity; outer loop disabled"),
                ("t/T", "Enter duration for BOTH-wheel tests [s]"),
                ("s/S", "Stop motion; remain in program"),
                ("e/E", "Exit program safely")))

        self.line_menu_lines = self._menu("ME 4305 Line Following Menu", (
            ("b/B", "Enter base wheel velocity [counts/s]"),
            ("p/P", "Enter outer P gain [(counts/s)/mm]"),
            ("t/T", "Enter finite run duration [s]"),
            ("g/G", "Start line following and collect data"),
            ("s/S", "Stop motion and transfer acquired data"),
            ("h/H", "Return to main menu"),
            ("e/E", "Exit program safely")))

    def wait_key(self, choices):
        while True:
            if self.vcp.any():
                raw = self.vcp.read(1)
                if raw:
                    key = chr(raw[0]).lower()
                    if key in ("e", "q"):
                        raise KeyboardInterrupt
                    if key in choices:
                        return key
            delay(10)  # Allowed here: scheduler has not started.

    def calibrate_startup(self, allow_motor_only=True):
        while True:
            print("\nStartup calibration: c = capture, u = reuse saved file, e = exit")
            if allow_motor_only:
                print("m = motor tests only (line following disabled)")
            key = self.wait_key("cum" if allow_motor_only else "cu")
            if key == "m":
                return False
            try:
                if key == "u":
                    print("Reuse only if paper, height, lighting and emitter settings are unchanged.")
                    print("y = confirm unchanged, c = capture again")
                    if self.wait_key("yc") == "c":
                        key = "c"
                    else:
                        calibration = load_calibration(
                            cfg.CALIBRATION_FILE, cfg.CHANNELS, cfg.MIN_SPAN)
                if key == "c":
                    orientation = cfg.CHANNEL_1_ON_LEFT
                    if orientation is None:
                        print("Which end is on Romi's LEFT? 1 = channel 1, 3 = channel 13")
                        orientation = self.wait_key("13") == "1"
                    self.sensor.set_orientation(orientation)
                    print("Place ALL 13 sensors over BLACK on the track paper; press b.")
                    self.wait_key("b")
                    black = self.sensor.capture()
                    print("Place ALL 13 sensors over WHITE on the same paper; press w.")
                    self.wait_key("w")
                    white = self.sensor.capture()
                    calibration = {"channels": list(cfg.CHANNELS), "black": black,
                                "white": white, "acquisition": self.sensor.acquisition(),
                                "channel_1_on_left": orientation}
                    self.sensor.apply_calibration(calibration)
                    save_calibration(cfg.CALIBRATION_FILE, calibration,
                                    cfg.CHANNELS, cfg.MIN_SPAN)
                    print("Saved " + cfg.CALIBRATION_FILE)
                self.sensor.apply_calibration(calibration)
                print("Calibration ready. Positive centroid must point to Romi's LEFT.")
                return True
            except (OSError, ValueError, TypeError, KeyError, RuntimeError) as error:
                self.sensor.calibrated = False
                print("Calibration unavailable: {}. Correct and try again.".format(error))

    
    def _menu(self, title, entries):
        lines = ["+" + "-" * 76 + "+",
                 "| {:<74} |".format(title),
                 "+" + "-" * 5 + "+" + "-" * 70 + "+"]
        for command, description in entries:
            lines.append("| {:<3} | {:<68} |".format(command, description))
        lines.append("+" + "-" * 5 + "+" + "-" * 70 + "+")
        return tuple(lines)

    def print_menu(self):
        if self.menu_index == 0:
            print()  # Move the border below the existing >: prompt.
        if self.menu_index < len(self.menu_lines):
            print(self.menu_lines[self.menu_index])
            self.menu_index += 1
            return False
        self.print_prompt()
        self.menu_index = 0
        return True

    def print_gain_menu(self):
        if self.gain_menu_index == 0:
            print()
        if self.gain_menu_index < len(self.gain_menu_lines):
            print(self.gain_menu_lines[self.gain_menu_index])
            self.gain_menu_index += 1
            return False
        print("Select p/i/f for gains or v for setpoint velocity:")
        self.gain_menu_index = 0
        self.gain_menu_waiting = True
        return True

    def print_startup_menu(self):
        # main calls this before creating periodic tasks.
        print()
        for line in self.menu_lines:
            print(line)
        self.print_prompt()
        self.state = S1_INPUT
        self.menu_index = 0

    def print_prompt(self):
        print(">: ", end="")

    def print_notice(self, message):
        print(message)

    def print_task_profile(self, profile):
        # Called only after main has stopped the scheduler and both motors.
        print("\nTask Profile:")
        print(profile)

    def read_command(self):
        if self.vcp.any():
            raw = self.vcp.read(1)
            if raw:
                return chr(raw[0]).lower()
        return None

    def _next_command(self, incoming):
        if self.pending_commands:
            if incoming is not None and len(self.pending_commands) < 128:
                self.pending_commands.append(incoming)
            return self.pending_commands.pop(0)
        return incoming

    def _can_start(self):
        return not (self.left_flag.run_flag or self.right_flag.run_flag
                    or self.left_print.print_flag or self.right_print.print_flag
                    or self.line.request is not None or self.line.active
                    or self.line_print.print_flag)

    def start_test(self, name, data, run_flag, print_flag):
        self.paired_output = False
        self.data, self.flag, self.print = data, run_flag, print_flag
        self.row_index, self.header_index = 0, 0
        self.header_started = self.csv_printed = False
        self.pending_commands.clear()
        self.flag.stop_requested = False
        self.flag.run_flag = True
        print("\n{}-hand Motor Step Response".format(name))
        self.state = S2_WAIT

    def _select_motor(self, command):
        if not self._can_start():
            return
        if self.loop_type.value == "closed":
            bank = self.gain_banks["left" if command == "l" else "right"]
            configured = (self.V_sp.value is not None
                          and (bank.K_P.value > 0 or bank.K_I.value > 0))
            if not configured:
                print("Must input a setpoint velocity and gain")
                return
        elif self.duty_cycle.value is None:
            print("Must input a duty cycle")
            return
        if command == "l":
            self.start_test("Left", self.left_data, self.left_flag, self.left_print)
        elif command == "r":
            self.start_test("Right", self.right_data, self.right_flag, self.right_print)

    def digit_entry(self, target, prompt):
        self.char_buf.clear()
        self.entry_overflow = False
        self.input_target = target
        self.entry_return_state = self.state
        self.state = S4_DIGIT
        print(prompt, end="")

    def _finish_entry(self):
        self.vcp.write("\r\n")
        text = "".join(self.char_buf).strip()
        try:
            if self.entry_overflow:
                raise ValueError("Entry is too long")
            if not text:
                self.vcp.write("Value not changed\r\n")
            else:
                # Permit signed decimal notation, but reject malformed input.
                if any(char not in "0123456789.+-" for char in text):
                    raise ValueError("Use decimal notation")
                value = float(text)
                if not isfinite(value):
                    raise ValueError("Value must be finite")
                if self.input_target == "duty_cycle":
                    if not -100 < value < 100:
                        raise ValueError("Duty cycle must be between -100 and 100")
                    self.duty_cycle.value = value
                    self.vcp.write("Value set to {}\r\n".format(value))
                elif self.input_target in ("K_P", "K_I", "K_FF"):
                    bound = {"K_P": self.settings.kp_max, "K_I": self.settings.ki_max,
                             "K_FF": self.settings.kff_max}[self.input_target]
                    if not 0 <= value < bound:
                        raise ValueError("Gain must be nonnegative and below {}".format(bound))
                    banks = (self.gain_banks.values() if self.gain_target == "both"
                             else (self.gain_banks[self.gain_target],))
                    for bank in banks:
                        getattr(bank, self.input_target).value = value
                    label = {"K_P": "KP", "K_I": "KI", "K_FF": "KFF"}[self.input_target]
                    self.vcp.write("{} set to {}\r\n".format(label, value))
                elif self.input_target in ("base", "outer_gain", "duration"):
                    if self.input_target == "base":
                        if not 0 < value < self.settings.wheel_speed_max:
                            raise ValueError("Base must lie inside configured wheel range")
                        self.line.base.value = value
                    elif self.input_target == "outer_gain":
                        if not 0 <= value < self.settings.line_gain_max:
                            raise ValueError("Outer gain outside entry range")
                        self.line.gain.value = value
                    else:
                        if not 0 < value <= self.settings.line_max_duration_s:
                            raise ValueError("Duration outside configured range")
                        self.line.duration.value = value
                    self.vcp.write("{} set to {}\r\n".format(self.input_target, value))
                elif self.input_target == "V_sp":
                    bound = self.settings.setpoint_max
                    if not -bound < value < bound:
                        raise ValueError("Velocity must be between {} and {}".format(
                            -bound, bound))
                    self.V_sp.value = value
                    self.vcp.write("Velocity set to {}\r\n".format(value))
        except (ValueError, OverflowError) as error:
            self.vcp.write("Invalid entry: {}\r\n".format(error))
            self.vcp.write("Value not changed\r\n")

        # Empty, incomplete, invalid, and successful entries all leave S4.
        self.state = self.entry_return_state
        self.char_buf.clear()
        self.entry_overflow = False
        self.print_prompt()

    def _build_header(self):
        lines = ["# BEGIN TEST"]
        for key, value in self.data.metadata.items():
            lines.append("# {}: {}".format(key, value))
        lines.append("# status: {}".format(self.data.status))
        if self.data.reason:
            lines.append("# reason: {}".format(self.data.reason))
        lines.append("# END HEADER")
        return tuple(lines)

    def _finish_output(self, marker):
        self.print.print_flag = False
        self.data = self.flag = self.print = None
        self.header_lines = ()
        self.row_index = 0
        # Be ready BEFORE emitting the marker: the PC can send k immediately.
        self.state = S1_INPUT
        self.paired_output = False
        print(marker)
        self.print_prompt()

    def report_shutdown(self, reason="exit", message=""):
        # main calls this only after disabling both motors.
        if self.data is not None:
            self.data.status = "failed" if reason == "failed" else "aborted"
            self.data.reason = message or "Program stopped before transfer completed"
            if not self.header_started:
                print()
                for line in self._build_header():
                    print(line)
            marker = "Test failed" if reason == "failed" else "Test aborted"
            print("{}: {}".format(marker, self.data.reason))
        elif reason == "failed":
            print("\nTest failed: {}".format(message))
        print("\nExiting Program")

    def _start_pair(self, mode):
        if not self._can_start():
            return
        if any(bank.K_P.value == 0 and bank.K_I.value == 0
               for bank in self.gain_banks.values()):
            print("Each wheel needs a positive K_P or K_I")
            return
        if mode == "line" and not self.sensor.calibrated:
            print("Restart and calibrate before line following")
            self.print_prompt()
            return
        if mode == "straight":
            if self.V_sp.value is None or not 0 < self.V_sp.value <= self.settings.wheel_speed_max:
                print("Enter a positive velocity inside the configured wheel range")
                return
            self.line.straight_speed = self.V_sp.value
        self.line.stop = False
        self.line.request = mode
        self.data, self.flag, self.print = self.line_data, None, self.line_print
        self.paired_output = True
        self.header_started = self.csv_printed = False
        self.header_index = self.row_index = 0
        self.pending_commands.clear()
        self.state = S2_WAIT
        print("\nPaired-wheel run: " + mode)

    def _stop_motion(self):
        for flag in (self.left_flag, self.right_flag):
            if flag.run_flag:
                flag.stop_requested = True
        if self.line.active or self.line.request is not None:
            self.line.stop = True
        self.pending_commands.clear()
        if self.state == S4_DIGIT:
            self.char_buf.clear()
            self.state = S1_INPUT
            self.print_prompt()

    def run(self):
        while True:
            # Check if UI runs while line task happens
            if self.line.running:
                self.motion_input_checks += 1

            incoming = self.read_command()

            if incoming == "e" or self.exit_flag.exit_flag:
                # Yield immediately; main stops motors before printing status.
                self.exit_flag.exit_flag = True
                yield self.state
                continue

            if incoming == "s":
                self._stop_motion()
                yield self.state
                continue

            if self.state == S0_INIT:
                if incoming is not None and len(self.pending_commands) < 128:
                    self.pending_commands.append(incoming)
                if self.print_menu():
                    self.state = S1_INPUT

            elif self.state == S1_INPUT:
                command = self._next_command(incoming)
                if command == "h":
                    self.loop_type.value = "open"
                    self.menu_index = 0
                    self.state = S0_INIT
                elif command == "d":
                    self.loop_type.value = "open"
                    self.digit_entry("duty_cycle", "Enter duty cycle [%]: ")
                elif command == "n":
                    self.line_menu_index = 0
                    self.line_menu_waiting = False
                    self.state = S6_LINE
                elif command == "k":
                    self.loop_type.value = "closed"
                    self.gain_menu_waiting = False
                    self.gain_menu_index = 0
                    self.state = S5_PARAM
                elif command in ("l", "r") and self.loop_type.value == "open":
                    self._select_motor(command)

            elif self.state == S2_WAIT:
                # Ignore other commands; print-ready takes priority over finished.
                if self.print.print_flag:
                    self.header_lines = self._build_header()
                    self.header_index = self.row_index = 0
                    self.csv_printed = False
                    self.state = S3_PRINT
                # Completion/stop always waits for the owner's data-ready acknowledgement.

            elif self.state == S3_PRINT:
                if self.header_index < len(self.header_lines):
                    print(self.header_lines[self.header_index])
                    self.header_started = True
                    self.header_index += 1
                elif self.data.status != "complete" and not self.paired_output:
                    marker = ("Test aborted" if self.data.status == "aborted"
                              else "Test failed")
                    self._finish_output("{}: {}".format(marker, self.data.reason))
                elif not self.csv_printed:
                    closed = self.data.metadata["loop_type"] == "closed"
                    print(LINE_HEADER if self.paired_output else (CLOSED_HEADER if closed else OPEN_HEADER))
                    self.csv_printed = True
                elif self.row_index < self.data.count:
                    index = self.row_index
                    if self.paired_output:
                        print(",".join(str(column[index]) for column in self.data.columns))
                    elif self.data.metadata["loop_type"] == "closed":
                        print("{},{},{},{},{}".format(
                            self.data.time[index], self.data.pos[index],
                            self.data.setpoint[index], self.data.vel[index],
                            self.data.voltage[index]))
                    else:
                        print("{},{},{}".format(
                            self.data.time[index], self.data.pos[index],
                            self.data.vel[index]))
                    self.row_index += 1
                else:
                    self._finish_output("End of data")

            elif self.state == S4_DIGIT:
                command = self._next_command(incoming)
                if command in self.term:
                    self._finish_entry()
                elif command in ("\x08", "\x7f"):
                    if self.char_buf:
                        self.char_buf.pop()
                        self.vcp.write("\b \b")
                elif command is not None and " " <= command <= "~":
                    if len(self.char_buf) < 64:
                        self.char_buf.append(command)
                        self.vcp.write(command)
                    else:
                        self.entry_overflow = True

            elif self.state == S5_PARAM:
                if not self.gain_menu_waiting:
                    if incoming is not None and len(self.pending_commands) < 128:
                        self.pending_commands.append(incoming)
                    self.print_gain_menu()
                else:
                    command = self._next_command(incoming)
                    if command in ("a", "1", "2"):
                        self.gain_target = {"a": "both", "1": "left", "2": "right"}[command]
                        print("Gain target: " + self.gain_target)
                        self.print_prompt()
                    elif command == "i":
                        self.digit_entry("K_I", "Enter integral gain Ki: ")
                    elif command == "f":
                        self.digit_entry("K_FF", "Enter feedforward gain Kff: ")
                    elif command == "t":
                        self.digit_entry("duration", "Enter run duration [s]: ")
                    elif command == "b":
                        self._start_pair("straight")
                    elif command == "p":
                        self.digit_entry("K_P", "Enter proportional gain Kp: ")
                    elif command == "v":
                        self.digit_entry("V_sp", "Enter setpoint velocity [counts/s]: ")
                    elif command == "h":
                        self.loop_type.value = "open"
                        self.menu_index = 0
                        self.state = S0_INIT
                    elif command in ("l", "r"):
                        self._select_motor(command)

            elif self.state == S6_LINE:
                if not self.line_menu_waiting:
                    if incoming is not None and len(self.pending_commands) < 128:
                        self.pending_commands.append(incoming)
                    if self.line_menu_index == 0:
                        print()
                    if self.line_menu_index < len(self.line_menu_lines):
                        print(self.line_menu_lines[self.line_menu_index])
                        self.line_menu_index += 1
                    else:
                        self.line_menu_waiting = True
                        print("Select b for base, p for outer gain, t for duration, g to go:")
                else:
                    command = self._next_command(incoming)
                    if command == "b":
                        self.digit_entry("base", "Enter base velocity [counts/s]: ")
                    elif command == "p":
                        self.digit_entry("outer_gain", "Enter outer gain [(counts/s)/mm]: ")
                    elif command == "t":
                        self.digit_entry("duration", "Enter run duration [s]: ")
                    elif command == "g":
                        self._start_pair("line")
                    elif command == "h":
                        self.loop_type.value = "open"
                        self.menu_index = 0
                        self.state = S0_INIT

            else:
                raise ValueError("Invalid TaskUser state")

            yield self.state

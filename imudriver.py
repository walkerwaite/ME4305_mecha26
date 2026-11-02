"""BNO055 driver for Romi, ME 4305 Lab 0x07

IMUPLUS uses accelerometer + gyroscope only (no magnetic heading).
Read heading and chassis yaw rate in radians / rad/s for control.
Calibration file handling belongs in imu_calibration.py, not this driver.

Default assumes a flat, sensor-Z-UP mounting and unchanged BNO055 axes:
  Bosch heading increases clockwise -> negate for Romi positive-left.
  Sensor gyro Z increases counterclockwise -> positive Romi left turn.
Check BOTH signs by turning the actual Romi by hand before motor tests.

Pass an initialized pyb.I2C object. The caller owns I2C and RST pins.
"""

from micropython import const
from struct import calcsize, unpack_from
from math import pi
from time import sleep_ms


class BNO055:
    DEV_ADDR = const(0x28)       # ADR low; ADR high gives 0x29
    CHIP_ID = 0xA0       # Expected value returned by the senso
    # Registers on page 0
    

    CONFIG_MODE = const(0x00)
    IMUPLUS_MODE = const(0x08)
    UNIT_SEL_VALUE = const(0x00)  # degrees, degrees/s, m/s^2, Celsius
    CALIBRATION_BYTES = const(22)
    DEG_TO_RAD = pi / 180.0

    class reg:
        # Confirm the chip ID register is readable and correct.
        CHIP_ID = (const(0x00), b"<B")  # Address of CHIP_ID register
        # Page_ID can be an integer because it is passed to _write_byte()
        PAGE_ID = (const(0x07))  # Address of PAGE_ID register
        # Preserve the starter's (register address, struct format) scheme.
        # Each 16-bit value is LSB first, then MSB. Read as one burst.
        GYR_Z_LSB = (const(0x18), b"<B")
        GYR_Z_MSB = (const(0x19), b"<B")
        GYR_Z = (const(0x18), b"<h")

        # Starts at 0x1A, reads 3 signed 16 bit numbers in sequence
        EUL_HEADING_MSB = (const(0x1B), b"<B")
        EUL_HEADING_LSB = (const(0x1A), b"<B")
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")

        # Calibration status register: 2 bits each for system, gyro, accel, mag
        CALIB_STAT = (const(0x35), b"<B")

        # Unit selection register address: degrees, degrees/s, m/s^2, Celsius
        UNIT_SEL = const(0x3B)

        # Add Operating mode address
        OPR_MODE = const(0x3D)

        # Power mode register address: normal, low power, suspend
        PWR_MODE = const(0x3E) # ?

        # System trigger register address: reset, self-test, oscillator
        SYS_TRIGGER = const(0x3F) #?

        # Axis remap register addresses: config and sign
        AXIS_MAP_CONFIG = const(0x41)
        AXIS_MAP_SIGN = const(0x42)

        # First address of the 22-byte calibration profile block
        OFFSET_START = const(0x55)  # 0x55..0x6A, inclusive: 22 bytes

    def __init__(self, i2c, address=DEV_ADDR, reset_pin=None,
                 heading_sign=-1, gyro_sign=1,
                 axis_map=0x24, axis_sign=0x00, mount_id="flat_z_up"):
        


        self._i2c = i2c # declare i2c as a variable to be used in the class

        # Store the configuration parameters for later use and validation
        self.address = address
        self.heading_sign = heading_sign
        self.gyro_sign = gyro_sign
        self.axis_map = axis_map
        self.axis_sign = axis_sign
        self.mount_id = mount_id

        # Check device address
        if address not in (0x28, 0x29):
            raise ValueError("BNO055 I2C address must be 0x28 or 0x29")

        # Check that heading and gyro signs are either +1 or -1
        if heading_sign not in (-1, 1) or gyro_sign not in (-1, 1):
            raise ValueError("Heading and gyro signs must be +1 or -1")

        # Check that axis_map is a 6-bit value and axis_sign is a 3-bit value
        if not 0 <= axis_map <= 0x3F or not 0 <= axis_sign <= 0x07:
            raise ValueError("Invalid sensor axis mapping")

        # Check that mount_id is a non-empty string
        if not isinstance(mount_id, str) or not mount_id:
            raise ValueError("Provide a sensor mounting identifier")

        self._i2c = i2c
        self.address = address
        self.heading_sign = heading_sign
        self.gyro_sign = gyro_sign
        self.axis_map = axis_map
        self.axis_sign = axis_sign
        self.mount_id = mount_id
        self._buf = bytearray(self.CALIBRATION_BYTES)

        # Create a reusable 22-byte buffer for reading/writing calibration data
        self._buf = bytearray(self.CALIBRATION_BYTES)  # constant 22-byte buffer for calibration data

        # Optional RST line: HIGH is normal operation, LOW holds sensor reset.
        if reset_pin is not None:
            reset_pin.high()

        self._initialize()  # Must run BEFORE cooperative scheduling.

    def _read_bytes(self, address, length):
        # Read a block of bytes from the sensor into the reusable buffer.
        # Used for calibration data.
        if not 1 <= length <= len(self._buf):
            raise ValueError("Invalid IMU register read length")
        
        buf = memoryview(self._buf)[:length]
        self._i2c.mem_read(buf, self.address, address, timeout=100)

        return buf # Return the memoryview of the buffer containing the read bytes.


    def _read_reg(self, reg):
        # Read a specified register from the sensor and return the unpacked value(s).
        # Same buffered register read as the starter script.
        length = calcsize(reg[1]) # Determine number of bytes to read
        
        buf = memoryview(self._buf)[:length] # Create a memoryview object of the right size

        self._i2c.mem_read(buf, self.address, reg[0], timeout=5) # Read the specified register into the buffer

        # Return the unpacked data from the buffer according to the specified format in reg[1]    
        return unpack_from(reg[1], buf)

    def _write_bytes(self, address, data):
        # Write a block of bytes to the sensor at the specified register address.
        self._i2c.mem_write(data, self.address, address, timeout=5)

    def _write_byte(self, address, value):
        self._write_bytes(address, bytes((value,)))

    def _set_mode(self, mode):
        self._write_byte(self.reg.OPR_MODE, mode)
        sleep_ms(25 if mode == self.CONFIG_MODE else 10)

    def _initialize(self):
        # Account for the BNO055 power-on interval before accessing registers.
        sleep_ms(650) # Wait for the BNO055 sensor to power on and stabilize before accessing registers.
        # ESTELLE: I THINK THAT THIS CHIP ID CHECK IS UNNECESSARY.
        chip_id = None # initialize chip_id to None to ensure it is defined before the loop

        # Try to read the chip ID register up to 10 times, with a 100 ms delay between attempts.
        for _ in range(10):
            try:
                chip_id = self._read_reg(self.reg.CHIP_ID)[0]

                if chip_id == self.CHIP_ID:
                    break
            except OSError:
                pass
            sleep_ms(100)

        if chip_id != self.CHIP_ID:
            raise RuntimeError("BNO055 not detected; check I2C wiring/address")

        self._set_mode(self.CONFIG_MODE) # Method to write to the OPR_MODE register to set the sensor to CONFIG mode.
        # _set_mode() includes a sleep_ms() delay to allow the sensor to switch modes.

        # Write the configuration registers to set up the sensor for IMUPLUS mode.
        self._write_byte(self.reg.PAGE_ID, 0)             # Register page 0
        self._write_byte(self.reg.PWR_MODE, 0x00)         # Normal power
        sleep_ms(10)  # Allow power-mode transition during startup only.

        self._write_byte(self.reg.SYS_TRIGGER, 0x00)      # Internal oscillator
        self._write_byte(self.reg.UNIT_SEL, self.UNIT_SEL_VALUE) # Set units to degrees, degrees/s, m/s^2, Celsius
        self._write_byte(self.reg.AXIS_MAP_CONFIG, self.axis_map) # Set axis remap configuration
        self._write_byte(self.reg.AXIS_MAP_SIGN, self.axis_sign) # Set axis remap sign configuration

        # ESTELLE: I THINK THAT THIS SEPARATE CHECK IS UNNECESSARY.
        # Fail initialization if the registers do not match the driver units.
        for register, expected in ((self.reg.UNIT_SEL, self.UNIT_SEL_VALUE),
                                   (self.reg.AXIS_MAP_CONFIG, self.axis_map),
                                   (self.reg.AXIS_MAP_SIGN, self.axis_sign)):
            if self._read_reg((register, b"<B"))[0] != expected:
                raise RuntimeError("BNO055 configuration read-back failed")
            
        self._set_mode(self.IMUPLUS_MODE) # Set the sensor to IMUPLUS mode for operation. This mode uses the accelerometer and gyroscope only, without relying on the magnetometer for heading information.

    def euler(self):
        # From the starter: raw sensor heading, roll, pitch in DEGREES.
        head, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        # Divide all by 16 to convert from raw sensor units to degrees, then convert to radians for calculations.
        return (head / 16.0, roll / 16.0, pitch / 16.0)

    def get_heading(self):
        # Heading (rad), positive for a Romi left turn. 
        head = self._read_reg(self.reg.EUL_HEADING)[0] # Read the heading value (in first tuple) from the sensor

        # Convert to radians and apply sign before returning the heading value.
        return self.heading_sign * head * self.DEG_TO_RAD / 16.0

    def get_yaw_rate(self):
        # Chassis yaw rate (rad/s), positive left. Bosch: 16 LSB/(deg/s).
        z = self._read_reg(self.reg.GYR_Z)[0] # Read the gyroscope Z-axis value (in first tuple) from the sensor

        # Convert to radians per second and apply sign before returning the yaw rate value.
        return self.gyro_sign * z * self.DEG_TO_RAD / 16.0

    def calibration_status(self):
        # Return (system, gyro, accel, mag) statuses, each 0 to 3.
        value = self._read_reg(self.reg.CALIB_STAT)[0] # Read the calibration status value (in first tuple) from the sensor

        # >> operator shifts bits to the rights (two lowest positions)
        # & 3 keeps only the two lowest bits (0b11) to get the status for each component.
        return ((value >> 6) & 3, (value >> 4) & 3,
                (value >> 2) & 3, value & 3)

    def is_calibrated(self):
        # IMUPLUS requires gyro and accelerometer only, NOT magnetometer.
        # _ replaces the unused system and magnetometer statuses.
        _, gyro, accel, _ = self.calibration_status()

        return gyro == 3 and accel == 3 # Return True if both gyro and accelerometer are fully calibrated (status 3), otherwise return False.

    def configuration(self):
        # Metadata for validating saved calibration against the current setup.
        # Returns Python dict with IMU setup parameters.
        return {"chip_id": self.CHIP_ID, 
                "address": self.address,
                "mode": self.IMUPLUS_MODE, 
                "unit_sel": self.UNIT_SEL_VALUE,
                "axis_map": self.axis_map,
                "axis_sign": self.axis_sign,
                "heading_sign": self.heading_sign, 
                "gyro_sign": self.gyro_sign,
                "mount_id": self.mount_id}

    def get_calibration(self):
        # Read and return the full 22-byte calibration profile from the sensor.
        # Must be fully calibrated BEFORE reading profile in CONFIG mode.

        # is_calibrated() checks that both gyro and accelerometer are fully calibrated (status 3).
        if not self.is_calibrated():
            raise ValueError("Gyro and accelerometer must both reach 3")

        # Switch to CONFIG mode to read the calibration data from the sensor.
        self._set_mode(self.CONFIG_MODE)

        # Read 22 bytes starting from the OFFSET_START register and return them as a list.
        try:
            return list(self._read_bytes(self.reg.OFFSET_START,
                                         self.CALIBRATION_BYTES))
        
        # Switch back to IMUPLUS mode after reading calibration data.
        finally:
            self._set_mode(self.IMUPLUS_MODE)

    def set_calibration(self, offsets):
        # Write a full 22-byte calibration profile to the sensor.
        # File validation belongs in imu_calibration.py; enforce safe bytes here.

        # Validate that offsets is a list or tuple of 22 integers, each between 0 and 255.
        if (not isinstance(offsets, (list, tuple)) or
                len(offsets) != self.CALIBRATION_BYTES or
                any(type(x) is not int or x < 0 or x > 255 for x in offsets)):
            raise ValueError("IMU calibration must contain 22 byte values")

        # Switch to CONFIG mode to write the calibration data to the sensor.
        self._set_mode(self.CONFIG_MODE)
        try:
            # Write each byte of the offsets to the corresponding register starting from OFFSET_START.
            for i, value in enumerate(offsets):
                self._write_byte(self.reg.OFFSET_START + i, value)

            # Validate file length, byte range, and configuration before restoring.
            # ESTELLE: "IF THE FILE IS ABSENT, INVALID, OR INAPPROPRIATE, REPEAT CALIBRATION
            # WITH MOTORS DISABLED""   
            actual = list(self._read_bytes(self.reg.OFFSET_START,
                                           self.CALIBRATION_BYTES))
            
            if actual != list(offsets):
                raise RuntimeError("BNO055 calibration read-back mismatch")
        finally:
            self._set_mode(self.IMUPLUS_MODE)
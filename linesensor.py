"""13-channel QTR-HD-13A driver; all acquisition uses the same timed batch.

Channel order: 1..13. MCU pins are in romi_config.SENSOR_PINS.
Positions: +24,+20,...,-24 mm if channel 1 is on Romi's LEFT; reversed
otherwise. Positive centroid is always left. Returned arrays are reused:
copy values explicitly when keeping a calibration capture or history.
"""
from pyb import ADC, Pin, Timer, delay
from time import ticks_us, ticks_diff
from ulab import numpy as np
from line_calibration import validate_calibration
import romi_config as cfg


class LineSensor:
    def __init__(self):
        self.controls = tuple(Pin(getattr(Pin.cpu, p), Pin.OUT_PP, value=0)
                              for p in cfg.CTRL_PINS)
        self.timer = None
        self.calibrated = False
        self.channel_1_on_left = cfg.CHANNEL_1_ON_LEFT
        try:
            if not 0 < cfg.ADC_HZ <= 5000 or cfg.ADC_SAMPLES < 1:
                raise ValueError("Invalid ADC acquisition settings")
            self.adcs = tuple(ADC(getattr(Pin.cpu, p)) for p in cfg.SENSOR_PINS)
            self.timer = Timer(cfg.ADC_TIMER, freq=cfg.ADC_HZ)
            # Separate contiguous buffers avoid stepped/column-slice hazards.
            self.buffers = tuple(np.zeros(cfg.ADC_SAMPLES, dtype=np.uint16)
                                 for _ in cfg.CHANNELS)
            self.raw = np.zeros(13, dtype=np.float)
            self.weights = np.zeros(13, dtype=np.float)
            self.white = np.zeros(13, dtype=np.float)
            self.span = np.zeros(13, dtype=np.float)
            self.positions = np.zeros(13, dtype=np.float)
            self.last_read_us = self.max_read_us = 0
            self.total_weight = 0.0
            delay(2)
            for control in self.controls:
                control.high()
            delay(2)
        except Exception:
            self.close()
            raise

    def acquisition(self):
        return {"method": "read_timed_multi", "pins": list(cfg.SENSOR_PINS),
                "timer": cfg.ADC_TIMER, "frequency_hz": cfg.ADC_HZ,
                "samples_per_channel": cfg.ADC_SAMPLES,
                "emitters": "both_full_brightness"}

    def set_orientation(self, channel_1_on_left):
        if type(channel_1_on_left) is not bool:
            raise ValueError("Confirm the array's physical orientation")
        self.channel_1_on_left = channel_1_on_left
        for i in range(13):
            self.positions[i] = (24 - 4 * i) * (1 if channel_1_on_left else -1)

    def read_raw(self):
        start = ticks_us()
        if not ADC.read_timed_multi(self.adcs, self.buffers, self.timer):
            raise RuntimeError("ADC acquisition overrun")
        for i in range(13):
            self.raw[i] = np.mean(self.buffers[i])
        self.last_read_us = ticks_diff(ticks_us(), start)
        self.max_read_us = max(self.max_read_us, self.last_read_us)
        return self.raw

    def capture(self, batches=cfg.CALIBRATION_BATCHES):
        # Startup only. Accumulate into independent Python values; never save
        # an alias/view of self.raw, which the next acquisition overwrites.
        averages = [0.0] * 13
        for _ in range(batches):
            self.read_raw()
            for i in range(13):
                averages[i] += float(self.raw[i]) / batches
        return averages

    def apply_calibration(self, calibration):
        validate_calibration(calibration, cfg.CHANNELS, cfg.MIN_SPAN)
        if calibration.get("acquisition") != self.acquisition():
            raise ValueError("Acquisition settings changed: recalibrate")
        orientation = calibration.get("channel_1_on_left")
        if (cfg.CHANNEL_1_ON_LEFT is not None
                and orientation != cfg.CHANNEL_1_ON_LEFT):
            raise ValueError("Mounting orientation changed: recalibrate")
        self.set_orientation(orientation)
        for i in range(13):
            self.white[i] = calibration["white"][i]
            self.span[i] = calibration["black"][i] - calibration["white"][i]
        self.calibrated = True

    def read(self):
        if not self.calibrated:
            raise ValueError("Line sensor is not calibrated")
        self.read_raw()
        total = moment = 0.0
        for i in range(13):
            w = min(1.0, max(0.0, (float(self.raw[i]) - self.white[i]) / self.span[i]))
            self.weights[i] = w
            total += w
            moment += w * self.positions[i]
        self.total_weight = total
        return None if total <= 0 else moment / total

    def close(self):
        for control in self.controls:
            control.low()
        if self.timer is not None:
            self.timer.deinit()

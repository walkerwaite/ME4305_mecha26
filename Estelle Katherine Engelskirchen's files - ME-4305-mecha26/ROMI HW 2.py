import pyb

tim_3 = pyb.Timer(3, freq=20_000)

PWM_1 = tim_3.channel(1, pin=pyb.Pin.cpu.A6, mode=pyb.Timer.PWM) # PWM_1 
PWM_2 = tim_3.channel(2, pin=pyb.Pin.cpu.A7, mode=pyb.Timer.PWM) # PWM_2

PC8 = pyb.Pin(pyb.Pin.cpu.C8, mode=pyb.Pin.OUT_PP) # Enable RH
PC6 = pyb.Pin(pyb.Pin.cpu.C6, mode=pyb.Pin.OUT_PP) # Enable LH

PB6 = pyb.Pin(pyb.Pin.cpu.B6, mode=pyb.Pin.OUT_PP) # Direction RH
PC7 = pyb.Pin(pyb.Pin.cpu.C7, mode=pyb.Pin.OUT_PP) # Direction LH

tim_1 = pyb.Timer(1, period = 0xFFFF, prescaler = 0)
tim_1.channel(1, pin=pyb.Pin.cpu.A8, mode=pyb.Timer.ENC_AB) # Encoder RH 1
tim_1.channel(2, pin=pyb.Pin.cpu.A9, mode=pyb.Timer.ENC_AB) # Encoder RH 2

tim_2 = pyb.Timer(2, period = 0xFFFF, prescaler = 0)
tim_2.channel(1, pin=pyb.Pin.cpu.A0, mode=pyb.Timer.ENC_AB) # Encoder LH 1
tim_2.channel(2, pin=pyb.Pin.cpu.A1, mode=pyb.Timer.ENC_AB) # Encoder LH 2 

print(tim_1.counter()) # RH
print(tim_2.counter()) # LH

PC8.high()
PC6.high()

# Forward
PB6.low()
PC7.low()

PWM_1.pulse_width_percent(25)
PWM_2.pulse_width_percent(25)

pyb.delay(1000)

print(tim_1.counter()) # RH
print(tim_2.counter()) # LH

# Stop
PWM_1.pulse_width_percent(0)
PWM_2.pulse_width_percent(0)

pyb.delay(1000)

print(tim_1.counter()) # RH
print(tim_2.counter()) # LH

# Backward
PB6.high()
PC7.high()

PWM_1.pulse_width_percent(25)
PWM_2.pulse_width_percent(25)

pyb.delay(1000)

print(tim_1.counter()) # RH
print(tim_2.counter()) # LH

# Stop

PWM_1.pulse_width_percent(0)
PWM_2.pulse_width_percent(0)
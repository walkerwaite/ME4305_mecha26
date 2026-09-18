from pyb import Pin, Timer, delay  # import the Pin and Timer classes from the pyb module

class MotorDriver: # defines class called MotorDriver
    # initializes a newly created object's attributes and sets up its initial state
    def __init__(self, pwm_pin: Pin, dir_pin: Pin, nslp_pin: Pin, tim: Timer, chan: int): 
        # store a copy of each input parameter as an attribute of the future object (self)
        self._dir_pin = Pin(dir_pin, Pin.OUT_PP) # sets direction pin as output
        self._nslp_pin = Pin(nslp_pin, Pin.OUT_PP) # sets nslp pin as output
        self._pwm_chan = tim.channel(chan, pin=pwm_pin, mode=Timer.PWM, pulse_width_percent=0) # sets PWM channel with pin, mode, and PWM percent
        # chan = channel #
        # pin=pwm_pin routes channel's PWM signal to this pin
        # mode=Timer.PWM sets channel to PWM mode
        # pulse_width_percent=0 sets the initial duty cycle to 0% (motor off)
        # by doing this with self, we ensure that no other commands will change these initial settings

    def enable(self): # defines method called enable
        self.set_effort(0) # sets effort to 0 to put it in brake mode
        self._nslp_pin.high() # sets nslp enable pin high to enable

    def disable(self): # defines method called disable
        self._nslp_pin.low() # sets nslp pin low to disable

    def set_effort(self, effort: float): # defines method called set_effort and input: only accepts float
        # check if effort is within the range of -100 to 100 
        if effort > -100 and effort < 100:
            # check if effort is negative, if negative set direction pin to high to reverse
            # else, set direction pin low to go forward (OURS IS REVERSED)
            if effort < 0:
                self._dir_pin.high() # set direction pin high to reversep
            else:
                self._dir_pin.low() # set direction pin low to go forward
            self._pwm_chan.pulse_width_percent(abs(effort)) # set PWM duty cycle to absolute value of effort, now that we have determined the direction
        else:
            pass







from time import ticks_us, ticks_diff # use to get dt value
import pyb

class Encoder: # defines new class called encoder
    # initializes a newly created object's attributes and sets up its initial state
    def __init__(self, tim: int, chA_pin: pyb.Pin, chB_pin: pyb.Pin): # defines init to require timer #, channels A&B pins

        self._tim_n = pyb.Timer(tim, period = 0xFFFF, prescaler = 0) # sets prescalar and period
        self._tim_n.channel(1, pin=chA_pin, mode=pyb.Timer.ENC_AB) # sets channel A of timer to encode
        self._tim_n.channel(2, pin=chB_pin, mode=pyb.Timer.ENC_AB) # sets channel B of timer to encode

        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between the last two updates
        self.dt         = 0     # Amount of time between the last two updates
        self.ar         = 65535 # Auto Reload value where over/underflow occurs
        self.prev_time  = 0     # Let previous time start at 0
        self.current_time  = ticks_us() # Get current ticks value

    def update(self):
        # Runs one update step on the encoder's timer counter 
        # to keep track of the change in count and check for counter reload
        self.delta = self._tim_n.counter() - self.prev_count # computes delta between past and current pos
        current_time = ticks_us()
        if self.delta < -(self.ar + 1)/2: # if delta negative magnitude too large (overflow)
            self.delta += self.ar + 1 # add autoreload value + 1
        elif self.delta > (self.ar + 1)/2: # if delta positive magnitude too large (underflow)
            self.delta -= self.ar + 1 # subtract autoreload value - 1

        self.position += self.delta # add delta to position
        self.prev_count = self._tim_n.counter() # set new previous count
        self.dt = ticks_diff(current_time, self.prev_time ) / 1_000_000 # let dt be difference between prev and current
        # divide by 1,000,000 to convert microseconds to seconds
        self.prev_time = current_time # set current to previous

    def get_position(self):
        # Returns the most recently updated value of position as determined 
        # within the update() method
        return self.position

    def get_velocity(self):
        # Returns a measure of velocity using the the most recently updated
        # value of delta as determined within the update() method
        if self.dt == 0:
            return 0
        return self.delta/self.dt

    def zero(self):
        # Sets the present encoder position to zero and causes future updates
        # to measure with respect to the new zero position
        self.position = 0
        self.prev_count = self._tim_n.counter()
        self.delta = 0
        self.dt = 0
        self.prev_time = ticks_us()
        



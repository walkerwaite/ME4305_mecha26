import encoder
import motordriver
from time import ticks_us, ticks_diff
from pyb import Pin, Timer, delay




if __name__ == "__main__":
    # Both motors can use the same timer but different channels
    pwm_tim = Timer(3, freq=20_000)
    left_motor = motordriver.MotorDriver(Pin.cpu.A7, Pin.cpu.C7, Pin.cpu.C6, pwm_tim, 2)
    right_motor = motordriver.MotorDriver(Pin.cpu.A6, Pin.cpu.B6, Pin.cpu.C8, pwm_tim, 1)

    # Set up encoder pins
    encoder_1 = encoder.Encoder(2, Pin.cpu.A0, Pin.cpu.A1) # Encoder RH

    encoder_2 = encoder.Encoder(1, Pin.cpu.A8, Pin.cpu.A9) # Encoder LH 

    # Zeros the encoders at the starting position
    encoder_1.zero()
    encoder_2.zero()

    # Enables both motors from motordriver.py
    left_motor.enable()
    right_motor.enable()

    # Sets motor effort using pwm in motordriver.py
    left_motor.set_effort(-42)
    right_motor.set_effort(42)

    for _ in range(500): # for 0.01s in 5 seconds
        encoder_1.update() # update encoder
        encoder_2.update()

        if _ % 100 == 0: # if _ is divisible by 100, then print
          print(f"right position: {encoder_1.get_position()}") # Print RH position
          print(f"left position:  {encoder_2.get_position()}") # Print LH position
          print(f"right velocity: {encoder_1.get_velocity()}") # Print RH velocity
          print(f"left velocity:  {encoder_2.get_velocity()}") # Print LH velcocity

        delay(10)

    # Change left motor direction and disable right motor
    left_motor.set_effort(42)
    right_motor.disable()

    for _ in range(500): # for 0.1s in 5 seconds
        encoder_1.update() # update encoder
        encoder_2.update()

        if _ % 100 == 0: # if _ is divisible by 100, then print
          print(f"right position: {encoder_1.get_position()}") # Print RH position
          print(f"left position:  {encoder_2.get_position()}") # Print LH position
          print(f"right velocity: {encoder_1.get_velocity()}") # Print RH velocity
          print(f"left velocity:  {encoder_2.get_velocity()}") # Print LH velcocity

        delay(10)

    # Change left motor speed and enable right motor
    left_motor.set_effort(10)
    right_motor.enable()

    for _ in range(500): # for 0.1s in 5 seconds
        encoder_1.update() # update encoder
        encoder_2.update()

        if _ % 100 == 0: # if _ is divisible by 100, then print
          print(f"right position: {encoder_1.get_position()}") # Print RH position
          print(f"left position:  {encoder_2.get_position()}") # Print LH position
          print(f"right velocity: {encoder_1.get_velocity()}") # Print RH velocity
          print(f"left velocity:  {encoder_2.get_velocity()}") # Print LH velcocity

        delay(10)

    # Change left motor speed and right motor speed
    left_motor.set_effort(75)
    right_motor.set_effort(42)

    for _ in range(500): # for 0.1s in 5 seconds
        encoder_1.update() # update encoder
        encoder_2.update()

        if _ % 100 == 0: # if _ is divisible by 100, then print
          print(f"right position: {encoder_1.get_position()}") # Print RH position
          print(f"left position:  {encoder_2.get_position()}") # Print LH position
          print(f"right velocity: {encoder_1.get_velocity()}") # Print RH velocity
          print(f"left velocity:  {encoder_2.get_velocity()}") # Print LH velcocity

        delay(10)

    # Disable left motor and change right motor speed and direction
    left_motor.disable()
    right_motor.set_effort(-42)

    for _ in range(500): # for 0.1s in 5 seconds
        encoder_1.update() # update encoder
        encoder_2.update()

        if _ % 100 == 0: # if _ is divisible by 100, then print
          print(f"right position: {encoder_1.get_position()}") # Print RH position
          print(f"left position:  {encoder_2.get_position()}") # Print LH position
          print(f"right velocity: {encoder_1.get_velocity()}") # Print RH velocity
          print(f"left velocity:  {encoder_2.get_velocity()}") # Print LH velcocity

        delay(10)

    # Both motors on, one forward and one backward to require reload
    left_motor.enable()
    left_motor.set_effort(42)
    right_motor.set_effort(-42)

    for _ in range(2000): # for 0.1s in 20 seconds
        encoder_1.update() # update encoder
        encoder_2.update()

        if _ % 100 == 0: # if _ is divisible by 100, then print
          print(f"right position: {encoder_1.get_position()}") # Print RH position
          print(f"left position:  {encoder_2.get_position()}") # Print LH position
          print(f"right velocity: {encoder_1.get_velocity()}") # Print RH velocity
          print(f"left velocity: {encoder_2.get_velocity()}") # Print LH velcocity

        delay(10)

    
    left_motor.disable()
    right_motor.disable()
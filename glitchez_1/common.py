#!/usr/bin/env pybricks-micropython

"""
Shared config values used across all modules
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Direction, Button, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
#obstacle_sensor = UltrasonicSensor(Port.S2)

# Initialize the color sensor for lines
line_sensor = ColorSensor(Port.S4)

# Initialize the color sensor for other stuff
color_sensor = ColorSensor(Port.S1)

gyro = GyroSensor(Port.S2, Direction.CLOCKWISE)

# 
arm = Motor(Port.A, Direction.CLOCKWISE)

CAN_DRIVE = False
HAVE_WHEEL_ARM = False


gyro.reset_angle(0)

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
try:

    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)

    CAN_DRIVE = True

except OSError:
    print("No drive motors attached.  Disabled driving.")
    ev3.speaker.say("I cannot drive")

# Initialize the treadmill arm and mark it as available
try:

    # Rotating Wheel 
    treadmill_motor = Motor(Port.D)
    HAVE_WHEEL_ARM = True

except OSError:
    print("No treadmill arm attached.  Disabled WHEEL_ARM.")
    #ev3.speaker.say("No arm")





def beeper():
    ev3.speaker.play_notes(['C4/4', 'C4/4', 'G4/4', 'G4/4'])


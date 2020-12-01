#!/usr/bin/env pybricks-micropython

"""
Shared config values used across all modules
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
#obstacle_sensor = UltrasonicSensor(Port.S2)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S4)

# 
arm = Motor(Port.A, Direction.CLOCKWISE)

# Rotating Wheel 
wheelie = Motor(Port.D)

CAN_DRIVE = False

# Initialize two motors with default settings on Port B and Port C.
# These will be the left and right motors of the drive base.
try:

    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)

    CAN_DRIVE = True

except OSError:
    print("No drive motors attached.  Disabled driving.")
    ev3.speaker.say("I cannot drive")


def beeper():
    ev3.speaker.play_notes(['C4/4', 'C4/4', 'G4/4', 'G4/4'])


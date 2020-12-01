#!/usr/bin/env pybricks-micropython

"""
FLL lego - arm handling
"""

import common

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase



#
# This function simply wags the arm around until the
# ultrasonic distance sensor doesn't detect
# anything within 100cm  (1000mm)
#
def wag_arm():

    common.ev3.light.on(Color.GREEN)

    common.arm.run_angle(100, -180)

    while common.obstacle_sensor.distance() < 1000:

        while common.obstacle_sensor.distance() < 300:
            common.ev3.light.on(Color.YELLOW)
            common.arm.run_angle(100, 120)
            common.arm.run_angle(100, -120)
            wait(10)

        common.ev3.light.on(Color.GREEN)
        common.arm.run_angle(100, 60)
        common.arm.run_angle(100, -60)

    common.ev3.light.on(Color.RED)
    common.arm.run_angle(100, 150)


def to_low_start():
    # move the arm down before starting
    common.arm.run_until_stalled(400)
    common.arm.run_angle(100, -20)

to_low_start()

common.ev3.speaker.say("arm is ready")

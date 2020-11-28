#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

arm_motor = Motor(Port.A, Direction.CLOCKWISE)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
treadmill_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=165)

ev3 = EV3Brick()
line_sensor = ColorSensor(Port.S4)


def go_bonkers(turns):
    half_turns = turns/2
    for i in range(half_turns):
        arm_motor.run_angle(100, 200, Stop.COAST, False)
        robot.turn(360)
        arm_motor.run_angle(-100, 200, Stop.COAST, False)
        robot.turn(-360)

go_bonkers(3)
#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait


ev3 = EV3Brick()

#treadmill_arm = Motor(Port.D)
pull_up_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=165.1, axle_track=150)
   
pull_up_motor.run_angle(-500, 250, Stop.BRAKE)
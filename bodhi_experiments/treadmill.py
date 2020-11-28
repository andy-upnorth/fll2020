#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait


# Change the brick light color and pause.
def flash(color, duration_ms):
    brick.light(color)
    wait(duration_ms)

def go_bonkers(turns):
    for i in range(turns):
        robot.turn(360)
        wait(1)


# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
flash(Color.RED, 200)
arm_motor = Motor(Port.A, Direction.CLOCKWISE)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
treadmill_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=165)
# axle_track=170 overshoots by about 15 degrees

# Initialize the sensors.
flash(Color.YELLOW, 200)
#distance_sensor = UltrasonicSensor(Port.S2)
line_sensor = ColorSensor(Port.S4)

# Ready to start.
flash(Color.RED, 400)
brick.light(None)

# Debug: Turn 360 degrees and pause for 2 seconds to manually re-aim robot
# if it no longer points in the right direction. Adjust DriveBase axle_track
# if a precise 360 wasn't performed.
#robot.turn(360)
#ev3.speaker.beep()
#wait(2000)
#ev3.speaker.beep()

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 5
WHITE = 45
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 3.0 # 1.2

# Drive forward up to 30 centimeters until the robot finds the line.
while line_sensor.reflection() > threshold:
    robot.drive(DRIVE_SPEED, 0)
ev3.speaker.beep()

treadmill_motor.set_dc_settings(30, 0)
treadmill_motor.run(-300)

# Follow line until arm motor stalls.
while (treadmill_motor.stalled() == False):
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold
    
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)
ev3.speaker.beep()

# Reset arm motor defaults.
treadmill_motor.stop()
robot.stop()
treadmill_motor.set_dc_settings(100, 0)

# Climb onto treadmill using all 3 wheels.
CLIMB_SPEED=100
CLIMB_ANGLE=120
treadmill_motor.run_angle(CLIMB_SPEED, CLIMB_SPEED, Stop.COAST, False)
left_motor.run_angle(CLIMB_SPEED, CLIMB_SPEED, Stop.COAST, False)
right_motor.run_angle(CLIMB_SPEED, CLIMB_SPEED, Stop.COAST, True)
ev3.speaker.beep()

# Spin the treadmill by a specified angle.
TREADMILL_SPEED=550
TREADMILL_ANGLE=1700
treadmill_motor.run_angle(TREADMILL_SPEED, TREADMILL_ANGLE, Stop.BRAKE)

# Climb onto treadmill using all 3 wheels.
treadmill_motor.run_angle(CLIMB_SPEED, -4 * CLIMB_ANGLE, Stop.COAST, False)
left_motor.run_angle(CLIMB_SPEED, -4 * CLIMB_ANGLE, Stop.COAST, False)
right_motor.run_angle(CLIMB_SPEED, -4 * CLIMB_ANGLE, Stop.COAST, False)

for i in range(3):
    ev3.speaker.beep(duration=750)
    wait(700)

robot.turn(360)
wait(300)
robot.turn(-360)

bonkers(6)
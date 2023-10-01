#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor
from pybricks.parameters import Port#, Color, ImageFile, SoundFile
from pybricks.tools import wait#, StopWatch
from pybricks.robotics import DriveBase
from pybricks.robotics import Stop

# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the motors connected to the drive wheels.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
color_sensor = ColorSensor(Port.S3)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)



# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold_line = (BLACK + WHITE) / 2

base_color=20
threshold_base=base_color
ev3.speaker.beep()

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = -30

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 2

# Initialize the gripper. First rotate the motor until it stalls.
# Stalling means that it cannot move any further. This position
# corresponds to the closed position. Then rotate the motor
# by 90 degrees such that the gripper is open.
gripper_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
gripper_motor.reset_angle(0) #gha bach n7ato reference des angles...
gripper_motor.run_target(200, -90)

while True:
    # Calculate the deviation from the threshold.
    line_deviation = color_sensor.reflection() - threshold_line
    # Calculate the turn rate.
    line_turn_rate = PROPORTIONAL_GAIN * line_deviation

    # Calculate the deviation from the threshold.
    base_deviation = color_sensor.reflection() - threshold_base
    # Calculate the turn rate.
    line_turn_rate = PROPORTIONAL_GAIN * base_deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, line_turn_rate)

    # You can wait for a short time or do other things in this loop.
    #wait(10)
    for i in range(1):
        ev3.speaker.beep()
    
    

    

    if base_deviation==0:
    # Open the gripper to release the wheel stack.
        for i in range(5):
            ev3.speaker.beep()
        gripper_motor.run_target(200, -90) #release the object
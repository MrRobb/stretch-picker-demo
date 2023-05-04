#!/usr/bin/env python3

import time
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad

r = stretch_body.robot.Robot()
if not r.startup():
    exit()  # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()


def doubt_what_to_pick():
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(0))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(0))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(0))

    time.sleep(1.0)

    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(45))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(0))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(45))

    time.sleep(1.0)

    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(45))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(0))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(-45))

    time.sleep(1.0)


def look_at_arm():
    r.head.move_to("head_pan", deg_to_rad(-90.0))
    r.head.move_to("head_tilt", deg_to_rad(-45.0))

    time.sleep(1.0)


def look_ahead():
    r.head.move_to("head_pan", 0)
    r.head.move_to("head_tilt", 0)

    time.sleep(1.0)


def retract_extend():
    r.arm.move_to(0.0)
    r.push_command()
    r.arm.wait_until_at_setpoint()

    r.arm.move_to(0.2)
    r.push_command()
    r.arm.wait_until_at_setpoint()


def robot_dance():
    # Move the robot's arm and wrist in a fun pattern
    r.arm.move_to(0.2)
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(30))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(45))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(-60))
    time.sleep(0.5)

    r.arm.move_to(0.0)
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(-30))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(-45))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(60))
    time.sleep(0.5)

    r.arm.move_to(0.2)
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(60))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(-30))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(45))
    time.sleep(0.5)

    r.arm.move_to(0.0)
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(-60))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(30))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(-45))
    time.sleep(0.5)

    # Move the robot's head in a fun pattern
    r.head.move_to("head_tilt", deg_to_rad(-30))
    time.sleep(0.5)
    r.head.move_to("head_tilt", deg_to_rad(30))
    r.head.move_to("head_pan", deg_to_rad(45))
    time.sleep(0.5)
    r.head.move_to("head_pan", deg_to_rad(-45))


def reset_arm():
    r.arm.move_to(0.0)
    r.arm.wait_until_at_setpoint()
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(0))
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(0))
    r.end_of_arm.move_to("wrist_yaw", deg_to_rad(0))
    time.sleep(1.0)


reset_arm()
look_at_arm()

retract_extend()
doubt_what_to_pick()

for i in range(5):
    robot_dance()

look_ahead()
reset_arm()

r.stop()

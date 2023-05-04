#!/usr/bin/env python3

import time
import stretch_body.robot

robot = stretch_body.robot.Robot()

robot.base.translate_by(x_m=0.5)
robot.push_command()
time.sleep(4.0)  # wait

robot.base.set_rotational_velocity(v_r=0.1)  # switch to velocity controller
robot.push_command()
time.sleep(4.0)  # wait

robot.base.set_rotational_velocity(v_r=0.0)  # stop motion
robot.push_command()

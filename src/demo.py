#!/usr/bin/env python3

import time
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad

r = stretch_body.robot.Robot()
if not r.startup():
    exit()  # failed to start robot!

# home the joints to find zero, if necessary
# if not r.is_calibrated():
#     r.home()

r.head.move_to("head_pan", 0)
r.head.move_to("head_tilt", 0)

time.sleep(3.0)

r.head.move_to("head_pan", deg_to_rad(-90.0))
r.head.move_to("head_tilt", deg_to_rad(-45.0))

time.sleep(3.0)

def retract_extend():
    r.arm.move_to(0.0)
    r.push_command()
    r.arm.wait_until_at_setpoint()

    r.arm.move_to(0.2)
    r.push_command()
    r.arm.wait_until_at_setpoint()


retract_extend()

r.end_of_arm.move_to("stretch_gripper", 50)

retract_extend()
r.end_of_arm.move_to("stretch_gripper", -50)
retract_extend()

r.stop()

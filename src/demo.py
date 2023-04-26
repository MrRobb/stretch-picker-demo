#!/usr/bin/env python3

import stretch_body.robot

r = stretch_body.robot.Robot()
if not r.startup():
    exit()  # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()

# interact with the robot here

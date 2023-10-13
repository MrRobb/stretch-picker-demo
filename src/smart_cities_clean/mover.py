#!/usr/bin/env python3

"""
This module handles the cartesian movement of the robot. It implements a ROS Action server that takes a string representing the object class and moves the robot gripper to the object.

INPUT
-----
The module takes the location of the object. It uses the stretch_body python module to move the robot gripper to the object.

OUTPUT
------
The module moves the robot gripper to the object.
"""

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

import rospy
from stretch_body.robot import Robot
from geometry_msgs.msg import Point

# Constants
TABLE_HEIGHT = 0.75
OBJECT_STRENGTH = {
    "bottle": -50,
    "remote": -50,
}
OBJECT_DEGREES = {
    "bottle": -10,
    "remote": -20,
}

# Globals
r = Robot()


if not r.startup():
    exit()  # failed to start robot!

# home the joints to find zero, if necessary
if not r.is_calibrated():
    r.home()


def move_to(msg: Point):
    """
    This function implements the pick action server. It takes a string representing the object class and uses the robot to pick the object.
    """
    rospy.loginfo(f"Moving to: {msg}")


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.loginfo("Starting pick node...")
    rospy.init_node("mover")

    # Create service server
    mover = rospy.Subscriber("move_to", Point, move_to, queue_size=1)

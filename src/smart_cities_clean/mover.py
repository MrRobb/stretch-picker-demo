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

import time
import math
import rospy
from stretch_body.robot import Robot
from geometry_msgs.msg import Point


class StretchMover:
    def __init__(self, robot: Robot, table_height: float = 70.0):
        self.r = robot
        self.table_height = table_height

        if not self.r.startup():
            rospy.logerr("Failed to start robot!")
            exit()

        # home the joints to find zero, if necessary
        if not self.r.is_calibrated():
            self.r.home()

    def reset_arm(self):
        self.r.arm.move_to(0.0)
        self.r.arm.wait_until_at_setpoint()
        self.r.end_of_arm.move_to("wrist_pitch", math.radians(0))
        self.r.end_of_arm.move_to("wrist_roll", math.radians(0))
        self.r.end_of_arm.move_to("wrist_yaw", math.radians(0))
        time.sleep(1.0)

    def move_to(self, msg: Point):
        """
        This function implements the pick action server. It takes a string representing the object class and uses the robot to pick the object.
        """
        rospy.loginfo(f"Moving to: {msg}")

        # Extract coordinates
        x_extend_inc = msg.x
        y_base_inc = msg.y
        z_lift_inc = msg.z

        # Move base
        self.r.base.translate_by(y_base_inc)
        time.sleep(0.1)
        self.r.push_command()
        self.r.base.wait_until_at_setpoint()

        # Lift
        self.r.lift.move_by(z_lift_inc)
        time.sleep(0.1)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        # Rotate wrist
        self.r.end_of_arm.move_to("wrist_yaw", math.radians(0))
        time.sleep(0.1)
        self.r.push_command()

        # Open gripper
        self.r.end_of_arm.pose("stretch_gripper", "open")
        time.sleep(0.1)
        self.r.push_command()
        time.sleep(1.0)

        # Move wrist down
        self.r.end_of_arm.move_to("wrist_pitch", math.radians(-20))
        time.sleep(0.1)
        self.r.push_command()

        # Extend arm
        self.r.arm.move_by(x_extend_inc)
        time.sleep(0.1)
        self.r.push_command()
        self.r.arm.wait_until_at_setpoint()

        # Close gripper
        self.r.end_of_arm.move_to("stretch_gripper", -50)
        time.sleep(0.1)
        self.r.push_command()
        time.sleep(2.0)

        # Lift up
        self.r.lift.move_by(0.2)
        time.sleep(0.1)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        # Rotate wrist
        self.r.end_of_arm.move_to("wrist_yaw", math.radians(-90))
        time.sleep(0.1)
        self.r.push_command()

        # Retract arm
        self.r.arm.move_to(0.0)
        time.sleep(0.1)
        self.r.push_command()
        self.r.arm.wait_until_at_setpoint()

        # Open gripper
        self.r.end_of_arm.pose("stretch_gripper", "open")
        time.sleep(0.1)
        self.r.push_command()
        time.sleep(2.0)

        # # Retract arm
        # self.r.arm.move_to(0.0)
        # time.sleep(0.1)
        # self.r.push_command()
        # self.r.arm.wait_until_at_setpoint()

        # # Move to table height
        # self.r.lift.move_to(self.TABLE_HEIGHT)
        # time.sleep(0.1)
        # self.r.push_command()
        # self.r.lift.wait_until_at_setpoint()

        # Lift down
        self.r.lift.move_to(self.table_height)
        time.sleep(0.1)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        self.reset_arm()

    def main(self):
        # Create subscriber to the pick topic
        rospy.Subscriber("move_to", Point, self.move_to, queue_size=1)


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.loginfo("Starting pick node...")
    rospy.init_node("mover")
    robot = Robot()
    mover = StretchMover(robot)
    mover.main()

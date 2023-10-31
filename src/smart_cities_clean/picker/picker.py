#!/usr/bin/env python3

"""
This module implements the picker functionality. Using a ROS Action, it implements the pick action server. The action server takes a string representing the object class and uses the robot to pick the object. 

INPUT
-----
The module takes a string representing the object class. Searches the object in the /detections topic. If the object is not found, it returns an error. If the object is found, it uses the mover module to move the robot gripper to the object.

OUTPUT
------
Sends the location of the object to the mover module.
"""

from typing import Dict, Optional
import rospy
import math
import numpy as np
from stretch_body.hello_utils import ThreadServiceExit
from picker_demo.srv import Picker, PickerRequest, PickerResponse
from visualization_msgs.msg import MarkerArray, Marker
import tf
from geometry_msgs.msg import Point, Pose, Quaternion
import stretch_body.robot
import time
from tf.listener import xyz_to_mat44, xyzw_to_mat44, transformations
from stretch_state_publisher import StretchStatePublisher


class StretchPicker:

    # Constants
    DETECTIONS: Dict[str, Marker] = {}
    temp_pub: Optional[rospy.Publisher] = None
    # Constants
    TABLE_HEIGHT = 0.7
    OBJECT_STRENGTH = {
        "bottle": -50,
        "remote": -50,
    }
    OBJECT_DEGREES = {
        "bottle": -10,
        "remote": -20,
    }

    #############################
    #       INITIALIZER         #
    #############################

    def __init__(self):
        self.r = stretch_body.robot.Robot()

        if not self.r.startup():
            exit()

        # home the joints to find zero, if necessary
        if not self.r.is_calibrated():
            self.r.home()

        # Set node name
        self.node_name = "picker"
        self.target_frame = "link_gripper"
        self.source_frame = "camera_color_optical_frame"

        # Set state manager
        self.state_manager = StretchStatePublisher(self.r)

        # TF Listener
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.target_frame,
            self.source_frame,
            rospy.Time(),
            rospy.Duration(4),
        )

    #############################
    #       TF FUNCTIONS        #
    #############################

    def transform(self, pose: Pose) -> Pose:

        (trans, rot) = self.listener.lookupTransform(
            self.target_frame, self.source_frame, rospy.Time(0)
        )

        mat44 = self.listener.fromTranslationRotation(trans, rot)

        # pose44 is the given pose as a 4x4
        pose44 = np.dot(
            xyz_to_mat44(pose.position),
            xyzw_to_mat44(pose.orientation),
        )

        # txpose is the new pose in target_frame as a 4x4
        txpose = np.dot(mat44, pose44)

        # xyz and quat are txpose's position and orientation
        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        return Pose(Point(*xyz), Quaternion(*quat))

    def publish_pose(self, pose: Pose):
        marker = Marker()

        marker.header.frame_id = self.target_frame
        marker.header.stamp = rospy.Time.now()

        marker.type = 2
        marker.id = 0

        marker.pose = pose

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        if self.temp_pub is not None:
            self.temp_pub.publish(marker)

    def rotate(self, p, origin=(0, 0), degrees=0):
        angle = np.deg2rad(degrees)
        R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.squeeze((R @ (p.T - o.T) + o.T).T)

    def calibrate(self, pose: Pose) -> Pose:
        # Rotate
        pose.position.x, pose.position.y = self.rotate(
            [pose.position.x, pose.position.y], degrees=5
        )
        return pose

    #############################
    #       ROBOT ACTIONS       #
    #############################

    def look_at_arm(self):
        self.r.head.move_to("head_pan", math.radians(-90.0))
        self.r.head.move_to("head_tilt", math.radians(-45.0))

        # self.r.head.move_to("head_pan", math.radians(-120.0))
        # self.r.head.move_to("head_pan", math.radians(-90.0))

        time.sleep(1.0)

    def look_ahead(self):
        self.r.head.move_to("head_pan", 0)
        self.r.head.move_to("head_tilt", 0)

        time.sleep(1.0)

    def reset_arm(self):
        self.r.arm.move_to(0.0)
        self.r.arm.wait_until_at_setpoint()
        self.r.end_of_arm.move_to("wrist_pitch", math.radians(0))
        self.r.end_of_arm.move_to("wrist_roll", math.radians(0))
        self.r.end_of_arm.move_to("wrist_yaw", math.radians(0))
        time.sleep(1.0)

    def pick_object(self, object_class: str):
        # Move to table height
        self.r.lift.move_to(self.TABLE_HEIGHT)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        # Open gripper
        self.r.end_of_arm.pose("stretch_gripper", "open")
        time.sleep(1.0)
        self.r.push_command()

        # Extend arm
        self.r.arm.move_to(0.5)
        time.sleep(1.0)
        self.r.push_command()
        self.r.arm.wait_until_at_setpoint()

        # Move wrist down
        self.r.end_of_arm.move_to(
            "wrist_pitch", math.radians(self.OBJECT_DEGREES[object_class])
        )
        time.sleep(1.0)
        self.r.push_command()

        # Close gripper
        self.r.end_of_arm.move_to("stretch_gripper", self.OBJECT_STRENGTH[object_class])
        time.sleep(1.0)
        self.r.push_command()
        time.sleep(1.0)

        # Move wrist up
        self.r.lift.move_to(self.TABLE_HEIGHT + 0.2)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

    def flip(self):

        # Move lift up
        print("\tMoving lift up")
        self.r.lift.move_to(self.TABLE_HEIGHT + 0.3)
        time.sleep(0.1)
        self.r.push_command()

        # Rotate wrist
        print("\tRotating wrist")
        self.r.end_of_arm.move_to("wrist_roll", math.radians(180))
        time.sleep(0.1)
        self.r.push_command()

        # Move lift down
        print("\tMoving lift down")
        self.r.lift.move_to(self.TABLE_HEIGHT)
        time.sleep(0.1)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

    def wiggle(self):
        # Move base to the right
        self.r.base.translate_by(0.1)
        self.r.push_command()
        self.r.base.wait_until_at_setpoint()

        # Move base to the left
        self.r.base.translate_by(-0.1)
        self.r.push_command()
        self.r.base.wait_until_at_setpoint()

    def leave_object(self):
        # Move to table height
        self.r.lift.move_to(self.TABLE_HEIGHT)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        # Open gripper
        self.r.end_of_arm.pose("stretch_gripper", "open")
        time.sleep(1.0)
        self.r.push_command()

        # Retract arm
        self.r.arm.move_to(0.0)
        time.sleep(1.0)
        self.r.push_command()
        self.r.arm.wait_until_at_setpoint()
        time.sleep(1.0)

    def robot_lift(self, object_class: str):
        self.pick_object(object_class)
        # flip()
        # wiggle()
        self.leave_object()

    #############################
    #       ROBOT STATES        #
    #############################

    def publish_state(self):
        self.state_manager.publish()

    #############################
    #       ROS CALLBACKS       #
    #############################

    def detections_callback(self, msg: MarkerArray):
        if msg.markers is None:
            return

        markers = [
            marker
            for marker in msg.markers
            if abs(marker.scale.x) >= 0.01 and abs(marker.scale.y) >= 0.01
        ]

        for marker in markers:
            self.DETECTIONS[marker.text] = marker
            if marker.text == "cup":
                self.publish_pose(self.transform(marker.pose))

        rospy.loginfo(f"DETECTIONS: {[ key for key in self.DETECTIONS.keys() ]}")

    def pick_callback(self, msg: PickerRequest) -> PickerResponse:
        """
        This function implements the pick action server. It takes a string representing the object class and uses the robot to pick the object.
        """
        rospy.loginfo(f"Picking: {msg.object_class}")

        # Retract arm
        self.r.arm.move_to(0.0)
        time.sleep(0.1)
        self.r.push_command()
        self.r.arm.wait_until_at_setpoint()

        # Move to table height
        self.r.lift.move_to(self.TABLE_HEIGHT)
        time.sleep(0.1)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        # Search for object in DETECTIONS
        # If object not found, return error
        if msg.object_class not in self.DETECTIONS:
            rospy.logerr(f"Object not found: {msg.object_class}")
            return PickerResponse(False)

        # If object found, call mover module
        rospy.loginfo(f"Found: {msg.object_class}")
        x = self.DETECTIONS[msg.object_class].pose.position.x
        y = self.DETECTIONS[msg.object_class].pose.position.y
        z = self.DETECTIONS[msg.object_class].pose.position.z
        rospy.loginfo(f"Original Location: {x}, {y}, {z}")

        # Get message
        detection_msg = self.DETECTIONS[msg.object_class]

        # Check source frame
        assert detection_msg.header.frame_id == self.source_frame

        # Publish pose
        transformed_pose = self.transform(detection_msg.pose)
        self.publish_pose(transformed_pose)

        # Calibrated pose
        calibrated_pose = self.calibrate(transformed_pose)

        y_base_inc = -calibrated_pose.position.y + 0.072
        z_lift_inc = calibrated_pose.position.z + 0.05 + 0.075
        x_extend_inc = -calibrated_pose.position.x - 0.2

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
        self.r.lift.move_to(self.TABLE_HEIGHT)
        time.sleep(0.1)
        self.r.push_command()
        self.r.lift.wait_until_at_setpoint()

        self.reset_arm()

        # Return success
        return PickerResponse(True)

    def main(self):
        # Create temporal publisher
        rospy.loginfo("Creating temporal publisher...")
        self.temp_pub = rospy.Publisher("/temp", Marker, queue_size=1)

        # Create service server
        rospy.loginfo("Creating detections subscriber...")
        rospy.Subscriber(
            "/objects/marker_array", MarkerArray, self.detections_callback, queue_size=1
        )

        print("Resetting arm...")
        self.reset_arm()
        print("Resetting arm... ✅")

        print("Looking at arm...")
        self.look_at_arm()
        print("Looking at arm... ✅")

        # Open gripper
        self.r.end_of_arm.pose("stretch_gripper", "open")
        time.sleep(0.1)
        self.r.push_command()
        time.sleep(1.0)

        rospy.loginfo("Creating service server...")
        rospy.Service("picker", Picker, self.pick_callback)

        # Set rate
        self.joint_state_rate = rospy.get_param("~rate", 30.0)
        rospy.loginfo("{0} rate = {1} Hz".format(self.node_name, self.joint_state_rate))

        # Set timeout
        self.timeout = rospy.get_param("~timeout", 0.5)
        rospy.loginfo("{0} timeout = {1} s".format(self.node_name, self.timeout))

        # Create rate
        rate = rospy.Rate(self.joint_state_rate)

        rospy.loginfo("Ready!")
        try:
            # start loop to command the mobile base velocity, publish
            # odometry, and publish joint states
            while not rospy.is_shutdown():
                self.r.non_dxl_thread.step()
                # self.r.pimu.set_fan_on()
                self.publish_state()
                self.r.push_command()
                self.dirty_command = False
                rate.sleep()
        except (rospy.ROSInterruptException, ThreadServiceExit):
            self.r.stop()
            rospy.signal_shutdown("stretch_driver shutdown")


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.loginfo("Starting pick node...")
    rospy.init_node("picker")
    picker = StretchPicker()
    picker.main()

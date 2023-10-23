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
import yaml
import tf_conversions
import numpy as np
import ikpy.chain
from stretch_body.hello_utils import ThreadServiceExit
from picker_demo.srv import Picker, PickerRequest, PickerResponse
from vision_msgs.msg import BoundingBox2D
from visualization_msgs.msg import MarkerArray, Marker
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad
import time
from geometry_msgs.msg import TransformStamped
from tf.listener import xyz_to_mat44, xyzw_to_mat44, transformations
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


#! /usr/bin/env python3

import importlib

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from joint_trajectory_server import JointTrajectoryAction


class StretchStatePublisher:
    def __init__(self, robot: stretch_body.robot.Robot):
        self.robot = robot

        large_ang = np.radians(45.0)
        filename = rospy.get_param("~controller_calibration_file")
        rospy.loginfo(
            "Loading controller calibration parameters for the head from YAML file named {0}".format(
                filename
            )
        )
        with open(str(filename), "r") as fid:
            self.controller_parameters = yaml.safe_load(fid)
            rospy.loginfo(
                "controller parameters loaded = {0}".format(self.controller_parameters)
            )

            head_tilt_calibrated_offset_rad = self.controller_parameters[
                "tilt_angle_offset"
            ]
            if abs(head_tilt_calibrated_offset_rad) > large_ang:
                rospy.logwarn(
                    "WARNING: head_tilt_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE"
                )
            rospy.loginfo(
                "head_tilt_calibrated_offset_rad in degrees = {0}".format(
                    np.degrees(head_tilt_calibrated_offset_rad)
                )
            )

            head_pan_calibrated_offset_rad = self.controller_parameters[
                "pan_angle_offset"
            ]
            if abs(head_pan_calibrated_offset_rad) > large_ang:
                rospy.logwarn(
                    "WARNING: head_pan_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE"
                )
            rospy.loginfo(
                "head_pan_calibrated_offset_rad in degrees = {0}".format(
                    np.degrees(head_pan_calibrated_offset_rad)
                )
            )

            head_pan_calibrated_looked_left_offset_rad = self.controller_parameters[
                "pan_looked_left_offset"
            ]
            if abs(head_pan_calibrated_looked_left_offset_rad) > large_ang:
                rospy.logwarn(
                    "WARNING: head_pan_calibrated_looked_left_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE"
                )
            rospy.loginfo(
                "head_pan_calibrated_looked_left_offset_rad in degrees = {0}".format(
                    np.degrees(head_pan_calibrated_looked_left_offset_rad)
                )
            )

            head_tilt_backlash_transition_angle_rad = self.controller_parameters[
                "tilt_angle_backlash_transition"
            ]
            rospy.loginfo(
                "head_tilt_backlash_transition_angle_rad in degrees = {0}".format(
                    np.degrees(head_tilt_backlash_transition_angle_rad)
                )
            )

            head_tilt_calibrated_looking_up_offset_rad = self.controller_parameters[
                "tilt_looking_up_offset"
            ]
            if abs(head_tilt_calibrated_looking_up_offset_rad) > large_ang:
                rospy.logwarn(
                    "WARNING: head_tilt_calibrated_looking_up_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE"
                )
            rospy.loginfo(
                "head_tilt_calibrated_looking_up_offset_rad in degrees = {0}".format(
                    np.degrees(head_tilt_calibrated_looking_up_offset_rad)
                )
            )

            arm_calibrated_retracted_offset_m = self.controller_parameters[
                "arm_retracted_offset"
            ]
            if abs(arm_calibrated_retracted_offset_m) > 0.05:
                rospy.logwarn(
                    "WARNING: arm_calibrated_retracted_offset_m HAS AN UNUSUALLY LARGE MAGNITUDE"
                )
            rospy.loginfo(
                "arm_calibrated_retracted_offset_m in meters = {0}".format(
                    arm_calibrated_retracted_offset_m
                )
            )

        self.broadcast_odom_tf = rospy.get_param("~broadcast_odom_tf", False)
        rospy.loginfo("broadcast_odom_tf = " + str(self.broadcast_odom_tf))

        if self.broadcast_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
        rospy.loginfo("odom_frame_id = " + str(self.odom_frame_id))

        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        rospy.loginfo("base_frame_id = " + str(self.base_frame_id))

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        rospy.loginfo("odom topic = " + str(self.odom_pub.name))

        self.fail_out_of_range_goal = rospy.get_param("~fail_out_of_range_goal", False)
        self.joint_trajectory_action = JointTrajectoryAction(self)
        self.joint_trajectory_action.server.start()

        self.voltage_history = []
        self.charging_state_history = [BatteryState.POWER_SUPPLY_STATUS_UNKNOWN] * 10
        self.charging_state = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.power_pub = rospy.Publisher("battery", BatteryState, queue_size=1)
        self.calibration_pub = rospy.Publisher("is_calibrated", Bool, queue_size=1)
        self.homed_pub = rospy.Publisher("is_homed", Bool, queue_size=1)
        self.mode_pub = rospy.Publisher("mode", String, queue_size=1)
        self.tool_pub = rospy.Publisher("tool", String, queue_size=1)
        self.imu_mobile_base_pub = rospy.Publisher("imu_mobile_base", Imu, queue_size=1)
        self.imu_wrist_pub = rospy.Publisher("imu_wrist", Imu, queue_size=1)
        self.runstop_event_pub = rospy.Publisher("is_runstopped", Bool, queue_size=1)
        self.magnetometer_mobile_base_pub = rospy.Publisher(
            "magnetometer_mobile_base", MagneticField, queue_size=1
        )

        self.robot_mode = "position"

    def publish(self):
        # TODO: Add rwlock

        # TODO: In the future, consider using time stamps from the robot's
        # motor control boards and other boards. These would need to
        # be synchronized with the rospy clock.
        # robot_time = robot_status['timestamp_pc']
        # current_time = rospy.Time.from_sec(robot_time)
        current_time = rospy.Time.now()
        robot_status = self.robot.get_status()

        ##################################################
        # obtain odometry
        base_status = robot_status["base"]
        x = base_status["x"]
        y = base_status["y"]
        theta = base_status["theta"]
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        x_vel = base_status["x_vel"]
        x_effort = base_status["effort"][0]
        theta_vel = base_status["theta_vel"]
        pose_time_s = base_status["pose_time_s"]

        if self.broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # publish odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_pub.publish(odom)

        ##################################################
        # obstain battery state
        pimu_hardware_id = self.robot.pimu.board_info["hardware_id"]
        invalid_reading = float("NaN")
        v = float(robot_status["pimu"]["voltage"])
        self.voltage_history.append(v)
        if len(self.voltage_history) > 100:
            self.voltage_history.pop(0)
            self.charging_state_history.pop(0)
            if v > np.mean(self.voltage_history) + 3 * np.std(self.voltage_history):
                self.charging_state_history.append(
                    BatteryState.POWER_SUPPLY_STATUS_CHARGING
                )
            elif v < np.mean(self.voltage_history) - 3 * np.std(self.voltage_history):
                self.charging_state_history.append(
                    BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                )
            else:
                self.charging_state_history.append(
                    BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                )
        filtered_charging_state = max(
            set(self.charging_state_history), key=self.charging_state_history.count
        )
        if filtered_charging_state != BatteryState.POWER_SUPPLY_STATUS_UNKNOWN:
            if pimu_hardware_id == 0:
                self.charging_state = filtered_charging_state
            elif pimu_hardware_id == 1:
                if (
                    robot_status["pimu"]["charger_connected"] == True
                    and filtered_charging_state
                    == BatteryState.POWER_SUPPLY_STATUS_CHARGING
                ):
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif (
                    robot_status["pimu"]["charger_connected"] == False
                    and filtered_charging_state
                    == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                ):
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            elif pimu_hardware_id == 2:
                if (
                    robot_status["pimu"]["charger_connected"] == True
                    and filtered_charging_state
                    == BatteryState.POWER_SUPPLY_STATUS_CHARGING
                ):
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif (
                    robot_status["pimu"]["charger_connected"] == False
                    and filtered_charging_state
                    == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                ):
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        i = float(robot_status["pimu"]["current"])
        if self.charging_state == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            i = float(robot_status["pimu"]["current"])
        elif self.charging_state == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
            i = -1 * float(robot_status["pimu"]["current"])

        # publish battery state
        battery_state = BatteryState()
        battery_state.header.stamp = current_time
        battery_state.voltage = v
        battery_state.current = i
        battery_state.temperature = invalid_reading
        battery_state.charge = invalid_reading
        battery_state.capacity = invalid_reading
        battery_state.percentage = invalid_reading  # TODO: Calculate the percentage
        battery_state.design_capacity = 18.0
        battery_state.power_supply_status = self.charging_state
        # misuse the 'present' flag to indicated whether the barrel jack button is pressed (i.e. charger is present, but may or may not be providing power)
        if pimu_hardware_id == 0:
            battery_state.present = False
        elif pimu_hardware_id == 1 or pimu_hardware_id == 2:
            battery_state.present = robot_status["pimu"]["charger_connected"]
        self.power_pub.publish(battery_state)

        ##################################################
        # publish homed status
        calibration_status = Bool()
        calibration_status.data = self.robot.is_calibrated()
        self.calibration_pub.publish(calibration_status)
        self.homed_pub.publish(calibration_status)

        # publish runstop event
        runstop_event = Bool()
        runstop_event.data = robot_status["pimu"]["runstop_event"]
        self.runstop_event_pub.publish(runstop_event)

        # publish mode status
        mode_msg = String()
        mode_msg.data = self.robot_mode
        self.mode_pub.publish(mode_msg)

        # publish end of arm tool
        tool_msg = String()
        tool_msg.data = self.robot.end_of_arm.name
        self.tool_pub.publish(tool_msg)

        ##################################################
        # publish joint state
        joint_state = JointState()
        joint_state.header.stamp = current_time
        cgs = list(
            set(self.joint_trajectory_action.command_groups)
            - set([self.joint_trajectory_action.mobile_base_cg])
        )
        for cg in cgs:
            pos, vel, effort = cg.joint_state(robot_status, robot_mode=self.robot_mode)
            joint_state.name.append(cg.name)
            joint_state.position.append(pos)
            joint_state.velocity.append(vel)
            joint_state.effort.append(effort)

        # add telescoping joints and wrist_extension to joint state
        arm_cg = self.joint_trajectory_action.arm_cg
        joint_state.name.extend(arm_cg.telescoping_joints)
        pos, vel, effort = arm_cg.joint_state(robot_status)
        for _ in range(len(arm_cg.telescoping_joints)):
            joint_state.position.append(pos / len(arm_cg.telescoping_joints))
            joint_state.velocity.append(vel / len(arm_cg.telescoping_joints))
            joint_state.effort.append(effort)
        joint_state.name.append(arm_cg.wrist_extension_name)
        joint_state.position.append(pos)
        joint_state.velocity.append(vel)
        joint_state.effort.append(effort)

        # add gripper joints to joint state
        gripper_cg = self.joint_trajectory_action.gripper_cg
        if gripper_cg is not None:
            missing_gripper_joint_names = list(
                set(gripper_cg.gripper_joint_names) - set(joint_state.name)
            )
            for j in missing_gripper_joint_names:
                pos, vel, effort = gripper_cg.joint_state(robot_status, joint_name=j)
                joint_state.name.append(j)
                joint_state.position.append(pos)
                joint_state.velocity.append(vel)
                joint_state.effort.append(effort)

        self.joint_state_pub.publish(joint_state)

        ##################################################
        # publish IMU sensor data
        imu_status = robot_status["pimu"]["imu"]
        ax = imu_status["ax"]
        ay = imu_status["ay"]
        az = imu_status["az"]
        gx = imu_status["gx"]
        gy = imu_status["gy"]
        gz = imu_status["gz"]
        mx = imu_status["mx"]
        my = imu_status["my"]
        mz = imu_status["mz"]

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = "imu_mobile_base"
        i.angular_velocity.x = gx
        i.angular_velocity.y = gy
        i.angular_velocity.z = gz
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_mobile_base_pub.publish(i)

        m = MagneticField()
        m.header.stamp = current_time
        m.header.frame_id = "imu_mobile_base"
        self.magnetometer_mobile_base_pub.publish(m)

        accel_status = robot_status["wacc"]
        ax = accel_status["ax"]
        ay = accel_status["ay"]
        az = accel_status["az"]

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = "accel_wrist"
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_wrist_pub.publish(i)
        ##################################################

        # TODO: self.robot_mode_rwlock.release_read()
        # # must happen after the read release, otherwise the write lock in change_mode() will cause a deadlock
        # if (self.prev_runstop_state == None and runstop_event.data) or (
        #     self.prev_runstop_state != None
        #     and runstop_event.data != self.prev_runstop_state
        # ):
        #     self.runstop_the_robot(runstop_event.data, just_change_mode=True)
        # self.prev_runstop_state = runstop_event.data


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

        # Set state manager
        self.state_manager = StretchStatePublisher(self.r)

    #############################
    #       ROBOT ACTIONS       #
    #############################

    def look_at_arm(self):
        self.r.head.move_to("head_pan", deg_to_rad(-90.0))
        self.r.head.move_to("head_tilt", deg_to_rad(-45.0))

        # self.r.head.move_to("head_pan", deg_to_rad(-120.0))
        # self.r.head.move_to("head_pan", deg_to_rad(-90.0))

        time.sleep(1.0)

    def look_ahead(self):
        self.r.head.move_to("head_pan", 0)
        self.r.head.move_to("head_tilt", 0)

        time.sleep(1.0)

    def reset_arm(self):
        self.r.arm.move_to(0.0)
        self.r.arm.wait_until_at_setpoint()
        self.r.end_of_arm.move_to("wrist_pitch", deg_to_rad(0))
        self.r.end_of_arm.move_to("wrist_roll", deg_to_rad(0))
        self.r.end_of_arm.move_to("wrist_yaw", deg_to_rad(0))
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
            "wrist_pitch", deg_to_rad(self.OBJECT_DEGREES[object_class])
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
        self.r.end_of_arm.move_to("wrist_roll", deg_to_rad(180))
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

        # Transform from /camera_color_optical_frame to /link_grasp_center
        detection_msg = self.DETECTIONS[msg.object_class]
        # rospy.loginfo(f"Detection: {detection_msg}")

        target_frame = "link_lift"

        listener = tf.TransformListener()
        listener.waitForTransform(
            target_frame,
            detection_msg.header.frame_id,
            rospy.Time(),
            rospy.Duration(4),
        )
        (trans, rot) = listener.lookupTransform(
            target_frame, detection_msg.header.frame_id, rospy.Time(0)
        )

        mat44 = listener.fromTranslationRotation(trans, rot)

        # pose44 is the given pose as a 4x4
        pose44 = np.dot(
            xyz_to_mat44(detection_msg.pose.position),
            xyzw_to_mat44(detection_msg.pose.orientation),
        )

        # txpose is the new pose in target_frame as a 4x4
        txpose = np.dot(mat44, pose44)

        # xyz and quat are txpose's position and orientation
        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose))

        # assemble return value PoseStamped
        rpose = PoseStamped()
        rpose.header.stamp = detection_msg.header.stamp
        rpose.header.frame_id = target_frame
        rpose.pose = Pose(Point(*xyz), Quaternion(*quat))

        x = rpose.pose.position.x
        y = rpose.pose.position.y
        z = rpose.pose.position.z
        rospy.loginfo(f"Transformed Location: {x}, {y}, {z}")

        # Publish
        marker = Marker()

        marker.header = rpose.header

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose = rpose.pose

        # Publish the marker
        if self.temp_pub is not None:
            self.temp_pub.publish(marker)

        # move(r.pose)
        # my_chain = ikpy.chain.Chain.from_urdf_file(
        #     "/home/hello-robot/catkin_ws/src/stretch_ros/stretch_description/urdf/exported_urdf/stretch.urdf"
        # )
        # target_vector = [r.pose.position.x, r.pose.position.y, r.pose.position.z]
        # angles = my_chain.inverse_kinematics(target_vector)
        # rospy.loginfo(f"Angles: {angles}")

        # Move base
        self.r.base.translate_by(-rpose.pose.position.y + 0.07)
        time.sleep(0.1)
        self.r.push_command()
        self.r.base.wait_until_at_setpoint()

        # # Lift
        # self.r.lift.move_by(rpose.pose.position.y)
        # time.sleep(0.1)
        # self.r.push_command()
        # self.r.lift.wait_until_at_setpoint()

        # # Rotate wrist
        # self.r.end_of_arm.move_to("wrist_yaw", deg_to_rad(0))
        # time.sleep(0.1)
        # self.r.push_command()

        # # Open gripper
        # self.r.end_of_arm.pose("stretch_gripper", "open")
        # time.sleep(0.1)
        # self.r.push_command()
        # time.sleep(1.0)

        # # Extend arm
        # self.r.arm.move_by(-rpose.pose.position.x - 0.23)
        # time.sleep(0.1)
        # self.r.push_command()
        # self.r.arm.wait_until_at_setpoint()

        # # Close gripper
        # self.r.end_of_arm.move_to("stretch_gripper", -50)
        # time.sleep(0.1)
        # self.r.push_command()
        # time.sleep(1.0)

        # # Lift
        # self.r.lift.move_to(self.TABLE_HEIGHT + 0.2)
        # time.sleep(0.1)
        # self.r.push_command()
        # self.r.lift.wait_until_at_setpoint()

        # # Rotate wrist
        # self.r.end_of_arm.move_to("wrist_yaw", deg_to_rad(90))
        # time.sleep(0.1)
        # self.r.push_command()

        # # Retract arm
        # self.r.arm.move_to(0.0)
        # time.sleep(0.1)
        # self.r.push_command()
        # self.r.arm.wait_until_at_setpoint()

        # # Open gripper
        # self.r.end_of_arm.pose("stretch_gripper", "open")
        # time.sleep(0.1)
        # self.r.push_command()
        # time.sleep(1.0)

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
                self.r.pimu.set_fan_on()
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

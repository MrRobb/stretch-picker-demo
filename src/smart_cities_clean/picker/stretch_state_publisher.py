#! /usr/bin/env python3

import rospy
import yaml
import tf2_ros
import numpy as np
import tf_conversions
import stretch_body.robot
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import TransformStamped
from joint_trajectory_server import JointTrajectoryAction
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField


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
            joint_state.position.append(pos)  # type: ignore
            joint_state.velocity.append(vel)  # type: ignore
            joint_state.effort.append(effort)  # type: ignore

        # add telescoping joints and wrist_extension to joint state
        arm_cg = self.joint_trajectory_action.arm_cg
        joint_state.name.extend(arm_cg.telescoping_joints)
        pos, vel, effort = arm_cg.joint_state(robot_status)
        for _ in range(len(arm_cg.telescoping_joints)):
            joint_state.position.append(pos / len(arm_cg.telescoping_joints))  # type: ignore
            joint_state.velocity.append(vel / len(arm_cg.telescoping_joints))  # type: ignore
            joint_state.effort.append(effort)  # type: ignore
        joint_state.name.append(arm_cg.wrist_extension_name)
        joint_state.position.append(pos)  # type: ignore
        joint_state.velocity.append(vel)  # type: ignore
        joint_state.effort.append(effort)  # type: ignore

        # add gripper joints to joint state
        gripper_cg = self.joint_trajectory_action.gripper_cg
        if gripper_cg is not None:
            missing_gripper_joint_names = list(
                set(gripper_cg.gripper_joint_names) - set(joint_state.name)
            )
            for j in missing_gripper_joint_names:
                pos, vel, effort = gripper_cg.joint_state(robot_status, joint_name=j)  # type: ignore
                joint_state.name.append(j)
                joint_state.position.append(pos)  # type: ignore
                joint_state.velocity.append(vel)  # type: ignore
                joint_state.effort.append(effort)  # type: ignore

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

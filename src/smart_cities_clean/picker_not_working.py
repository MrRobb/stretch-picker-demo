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
import numpy as np
from picker_demo.srv import Picker, PickerRequest, PickerResponse
from vision_msgs.msg import BoundingBox2D
from visualization_msgs.msg import MarkerArray, Marker
import tf
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad
import time
from tf.listener import xyz_to_mat44, xyzw_to_mat44, transformations

class_names = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "airplane",
    5: "bus",
    6: "train",
    7: "truck",
    8: "boat",
    9: "traffic light",
    10: "fire hydrant",
    11: "stop sign",
    12: "parking meter",
    13: "bench",
    14: "bird",
    15: "cat",
    16: "dog",
    17: "horse",
    18: "sheep",
    19: "cow",
    20: "elephant",
    21: "bear",
    22: "zebra",
    23: "giraffe",
    24: "backpack",
    25: "umbrella",
    26: "handbag",
    27: "tie",
    28: "suitcase",
    29: "frisbee",
    30: "skis",
    31: "snowboard",
    32: "sports ball",
    33: "kite",
    34: "baseball bat",
    35: "baseball glove",
    36: "skateboard",
    37: "surfboard",
    38: "tennis racket",
    39: "bottle",
    40: "wine glass",
    41: "cup",
    42: "fork",
    43: "knife",
    44: "spoon",
    45: "bowl",
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    50: "broccoli",
    51: "carrot",
    52: "hot dog",
    53: "pizza",
    54: "donut",
    55: "cake",
    56: "chair",
    57: "couch",
    58: "potted plant",
    59: "bed",
    60: "dining table",
    61: "toilet",
    62: "tv",
    63: "laptop",
    64: "mouse",
    65: "remote",
    66: "keyboard",
    67: "cell phone",
    68: "microwave",
    69: "oven",
    70: "toaster",
    71: "sink",
    72: "refrigerator",
    73: "book",
    74: "clock",
    75: "vase",
    76: "scissors",
    77: "teddy bear",
    78: "hair drier",
    79: "toothbrush",
}

# Constants
DETECTIONS: Dict[str, Marker] = {}
temp_pub: Optional[rospy.Publisher] = None
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

# Robot
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


def robot_lift(object_class: str):
    pick_object(object_class)
    flip()
    # wiggle()
    leave_object()


def pick_object(object_class: str):
    # Move to table height
    r.lift.move_to(TABLE_HEIGHT)
    r.push_command()
    r.lift.wait_until_at_setpoint()

    # Open gripper
    r.end_of_arm.pose("stretch_gripper", "open")
    time.sleep(1.0)
    r.push_command()

    # Extend arm
    r.arm.move_to(0.5)
    time.sleep(1.0)
    r.push_command()
    r.arm.wait_until_at_setpoint()

    # Move wrist down
    r.end_of_arm.move_to("wrist_pitch", deg_to_rad(OBJECT_DEGREES[object_class]))
    time.sleep(1.0)
    r.push_command()

    # Close gripper
    r.end_of_arm.move_to("stretch_gripper", OBJECT_STRENGTH[object_class])
    time.sleep(1.0)
    r.push_command()
    time.sleep(1.0)

    # Move wrist up
    r.lift.move_to(TABLE_HEIGHT + 0.2)
    r.push_command()
    r.lift.wait_until_at_setpoint()


def flip():

    # Move lift up
    print("\tMoving lift up")
    r.lift.move_to(TABLE_HEIGHT + 0.3)
    time.sleep(0.1)
    r.push_command()

    # Rotate wrist
    print("\tRotating wrist")
    r.end_of_arm.move_to("wrist_roll", deg_to_rad(180))
    time.sleep(0.1)
    r.push_command()

    # Move lift down
    print("\tMoving lift down")
    r.lift.move_to(TABLE_HEIGHT)
    time.sleep(0.1)
    r.push_command()
    r.lift.wait_until_at_setpoint()


def wiggle():
    # Move base to the right
    r.base.translate_by(0.1)
    r.push_command()
    r.base.wait_until_at_setpoint()

    # Move base to the left
    r.base.translate_by(-0.1)
    r.push_command()
    r.base.wait_until_at_setpoint()


def leave_object():
    # Move to table height
    r.lift.move_to(TABLE_HEIGHT)
    r.push_command()
    r.lift.wait_until_at_setpoint()

    # Open gripper
    r.end_of_arm.pose("stretch_gripper", "open")
    time.sleep(1.0)
    r.push_command()

    # Retract arm
    r.arm.move_to(0.0)
    time.sleep(1.0)
    r.push_command()
    r.arm.wait_until_at_setpoint()
    time.sleep(1.0)


def detections_callback(msg: MarkerArray):
    if msg.markers is None:
        return

    for marker in msg.markers:
        rospy.logdebug(f"{marker.text} in {marker.pose}")

        if abs(marker.scale.x) >= 0.01 and abs(marker.scale.y) >= 0.01:
            DETECTIONS[marker.text] = marker

            # if marker.text == "cup":
            #     rospy.loginfo(
            #         f"{marker.text} in {marker.pose.position.x}, {marker.pose.position.y}, {marker.pose.position.z}, {marker.scale.x}, {marker.scale.y}, {marker.scale.z}"
            #     )

    # rospy.loginfo(f"DETECTIONS: {DETECTIONS.keys()}")


def move(pose: Pose):
    print("Resetting arm...")
    reset_arm()
    print("Resetting arm... ✅")

    print("Looking at arm...")
    look_at_arm()
    print("Looking at arm... ✅")

    # Retract arm
    r.arm.move_to(0.0)
    time.sleep(0.1)
    r.push_command()
    r.arm.wait_until_at_setpoint()

    # Move to table height
    r.lift.move_to(TABLE_HEIGHT)
    time.sleep(0.1)
    r.push_command()
    r.lift.wait_until_at_setpoint()

    # Move to object
    r.base.translate_by(pose.position.x)
    time.sleep(0.1)
    r.push_command()
    r.base.wait_until_at_setpoint()

    # retract_extend()
    # doubt_what_to_pick()

    # robot_dance()

    print("Lifting arm...")
    reset_arm()
    robot_lift("bottle")
    print("Lifting arm... ✅")


def pick(msg: PickerRequest) -> PickerResponse:
    """
    This function implements the pick action server. It takes a string representing the object class and uses the robot to pick the object.
    """
    rospy.loginfo(f"Picking: {msg.object_class}")

    # Search for object in DETECTIONS
    # If object not found, return error
    if msg.object_class not in DETECTIONS:
        rospy.logerr(f"Object not found: {msg.object_class}")
        return PickerResponse(False)

    # If object found, call mover module
    rospy.loginfo(f"Found: {msg.object_class}")
    rospy.loginfo(
        f"Location: {DETECTIONS[msg.object_class].pose.position.x}, {DETECTIONS[msg.object_class].pose.position.y}, {DETECTIONS[msg.object_class].pose.position.z}"
    )

    # Transform from /camera_color_optical_frame to /link_straight_gripper
    detection_msg = DETECTIONS[msg.object_class]
    # rospy.loginfo(f"Detection: {detection_msg}")

    target_frame = "link_straight_gripper"

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
    r = PoseStamped()
    r.header.stamp = detection_msg.header.stamp
    r.header.frame_id = target_frame
    r.pose = Pose(Point(*xyz), Quaternion(*quat))

    rospy.loginfo(f"{r}")

    # Publish
    marker = Marker()

    marker.header = r.header

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
    marker.pose = r.pose

    # Publish the marker
    if temp_pub is not None:
        temp_pub.publish(marker)

    # move(r.pose)

    # Return success
    return PickerResponse(True)


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.loginfo("Starting pick node...")
    rospy.init_node("picker")

    # Create temporal publisher
    rospy.loginfo("Creating temporal publisher...")
    temp_pub = rospy.Publisher("/temp", Marker, queue_size=1)

    # Create service server
    rospy.loginfo("Creating detections subscriber...")
    detections = rospy.Subscriber(
        "/objects/marker_array", MarkerArray, detections_callback, queue_size=1
    )

    print("Resetting arm...")
    reset_arm()
    print("Resetting arm... ✅")

    print("Looking at arm...")
    look_at_arm()
    print("Looking at arm... ✅")

    rospy.loginfo("Creating service server...")
    picker = rospy.Service("picker", Picker, pick)

    rospy.loginfo("Ready!")
    rospy.spin()

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

from typing import Dict
import rospy
from picker_demo.srv import Picker, PickerRequest, PickerResponse
from vision_msgs.msg import Detection2DArray, BoundingBox2D

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
DETECTIONS: Dict[str, BoundingBox2D] = {}


def detections_callback(msg: Detection2DArray):
    if msg.detections is None:
        return

    for detection in msg.detections:
        if len(detection.results) > 0:
            rospy.loginfo(f"Location: {detection.bbox}")
            DETECTIONS[detection.results[0].id] = detection.bbox


def pick(msg: PickerRequest) -> PickerResponse:
    """
    This function implements the pick action server. It takes a string representing the object class and uses the robot to pick the object.
    """
    rospy.loginfo(f"Picking: {msg.object_class}")

    # Search for object in DETECTIONS
    # If object not found, return error
    if msg.object_class not in DETECTIONS:
        return PickerResponse(False)

    # If object found, call mover module
    rospy.loginfo(f"Found: {msg.object_class}")
    rospy.loginfo(f"Location: {DETECTIONS[msg.object_class]}")
    

    # Return success
    return PickerResponse(True)


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.loginfo("Starting pick node...")
    rospy.init_node("picker")

    # Create service server
    rospy.loginfo("Creating detections subscriber...")
    detections = rospy.Subscriber(
        "/objects/detected", Detection2DArray, detections_callback, queue_size=1
    )

    rospy.loginfo("Creating service server...")
    picker = rospy.Service("pick", Picker, pick)

    rospy.spin()

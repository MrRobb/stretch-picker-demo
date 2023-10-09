#!/usr/bin/env python3

from typing import Dict, List, Optional
import rospy
import cv2
import pandas as pd
import numpy as np
import torch
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap

# from octomap import OcTree
from cv_bridge import CvBridge
from actionlib import SimpleActionClient
from control_msgs.msg import PointHeadAction
from geometry_msgs.msg import PointStamped, PoseWithCovariance, Pose, Point, Pose2D
from control_msgs.msg import PointHeadGoal
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.encoding import Config
from image_geometry import PinholeCameraModel

# GLOBAL PARAMETERS (can be changed)
WINDOW_NAME = "Camera view"
# CAMERA_FRAME = "/xtion_rgb_optical_frame"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
CAMERA_TOPIC = "/camera/color/image_raw"
POINT_HEAD_ACTION = "/head_controller/point_head_action"
OCTOMAP_TOPIC = "/throttle_filtering_points/filtered_points"
MAX_ITERATIONS = 3

# GLOBAL VARIABLES (should not be changed)
POINT_HEAD_ACTION_CLIENT = None
LAST_IMAGE_TIMESTAMP = rospy.Time(0)
CAMERA_INTRINSICS = PinholeCameraModel()
YOLO_MODEL = torch.hub.load("ultralytics/yolov5", "yolov5s", pretrained=True)
PARAMETER_SERVER: Optional[Server] = None
DETECTED_OBJECTS: List[Detection2D] = []


def build_detection(df: pd.Series, image: Image) -> Detection2D:
    """
    Build detection message
    """
    (x_min, y_min, z_min) = CAMERA_INTRINSICS.projectPixelTo3dRay(
        (df["xmin"], df["ymin"])
    )
    (x_max, y_max, z_max) = CAMERA_INTRINSICS.projectPixelTo3dRay(
        (df["xmax"], df["ymax"])
    )
    xcenter = (x_min + x_max) / 2
    ycenter = (y_min + y_max) / 2
    zcenter = (z_min + z_max) / 2
    width = x_max - x_min
    height = y_max - y_min
    msg = Detection2D(
        header=image.header,
        results=[
            ObjectHypothesisWithPose(
                id=df["class"],
                score=df["confidence"],
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=xcenter, y=ycenter, z=zcenter),
                        orientation=None,
                    ),
                    covariance=list(np.zeros((36,), dtype=float)),
                ),
            )
        ],
        bbox=BoundingBox2D(
            center=Pose2D(x=xcenter, y=ycenter, theta=0.0),
            size_x=width,
            size_y=height,
        ),
        source_img=image,
    )
    return msg


def apply_yolo(image: Image):
    """
    Apply YOLOv7 to the image
    """
    if YOLO_MODEL is None:
        rospy.logerr("YOLOv7 model not loaded")
        return

    if image is None:
        rospy.logerr("Image is None")
        return

    # Save in the displayed image
    cv_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
    results = YOLO_MODEL(cv_image)
    results.render()

    # Columns: [xmin', 'ymin', 'xmax', 'ymax', 'confidence', 'class', 'name']
    df_results: pd.DataFrame = results.pandas().xyxy[0]
    global DETECTED_OBJECTS
    DETECTED_OBJECTS = [
        build_detection(row, image) for (_, row) in df_results.iterrows()
    ]

    print(list(df_results["name"]))

    return cv_image


def image_callback(image: Image):
    print(image.header.stamp)

    # Rotate image 90 degrees
    cv_image = CvBridge().imgmsg_to_cv2(image, "bgr8")
    cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)

    # Construct rospy image
    image = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")

    cv_image = apply_yolo(image=image)

    # Save image as PNG
    cv2.imwrite("image.png", cv_image)  # type: ignore


def create_point_head_client():
    """
    Creates a point head client
    """
    rospy.loginfo("Creating action client to head controller...")
    client = SimpleActionClient(POINT_HEAD_ACTION, PointHeadAction)

    for _ in range(MAX_ITERATIONS):
        if client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("Action client created")
            return client
        else:
            rospy.logwarn("Waiting for action server...")

    rospy.logerr("Action client not created")


def main():
    rospy.init_node("detect_objects")

    # Initialize parameters server
    rospy.loginfo("Initializing parameter server...")

    # Get camera intrinsics
    rospy.loginfo("Getting camera intrinsic parameters...")
    msg = rospy.wait_for_message(
        CAMERA_INFO_TOPIC, CameraInfo, timeout=rospy.Duration(10)
    )  # type: CameraInfo | None
    if msg is not None:
        rospy.loginfo("Camera intrinsic parameters received")
        global CAMERA_INTRINSICS
        CAMERA_INTRINSICS.fromCameraInfo(msg)
        print(CAMERA_INTRINSICS)
    else:
        rospy.logerr("Camera intrinsic parameters not received")

    # Create point head client
    rospy.loginfo("Creating point head action client...")
    global POINT_HEAD_ACTION_CLIENT
    POINT_HEAD_ACTION_CLIENT = create_point_head_client()
    if POINT_HEAD_ACTION_CLIENT is not None:
        rospy.loginfo("Point head action client created")
    else:
        rospy.logerr("Point head action client not created")

    rospy.Subscriber(CAMERA_TOPIC, Image, image_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()

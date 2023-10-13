#!/usr/bin/env python3

"""
This module implements the detection of the objects on the field of view of the camera. 

INPUT
-----
It takes the ROS /camera/rgb/image_raw topic and uses a YOLOv5s model to detect the objects.

OUTPUT
------
The module is implemented as a ROS node, and it publishes the detected objects on the /detections topic.
"""

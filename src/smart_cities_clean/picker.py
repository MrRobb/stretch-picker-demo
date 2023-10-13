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

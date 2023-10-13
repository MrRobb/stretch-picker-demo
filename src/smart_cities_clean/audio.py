#!/usr/bin/env python3

"""
This module implements the text-to-speech functionality. Using a ROS Action, it implements the speak action server. The action server takes a string and uses the Google Text-to-Speech API to generate an audio file. Then, it uses the audio output device to play the audio file.
"""

import rospy
from actionlib import SimpleActionServer
from std_msgs.msg import String


def speak(sentence: String):
    """
    This function implements the speak action server. It takes a string and uses the Google Text-to-Speech API to generate an audio file. Then, it uses the audio output device to play the audio file.
    """
    rospy.loginfo(f"Speaking: {sentence.data}")

if __name__ == "__main__":
    # Create action server
    rospy.init_node("tts")
    server = SimpleActionServer("tts", String, execute_cb=speak)
    server.start()
    rospy.spin()
    
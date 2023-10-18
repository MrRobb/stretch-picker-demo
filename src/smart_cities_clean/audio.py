#!/usr/bin/env python3

"""
This module implements the text-to-speech functionality. Using a ROS Action, it implements the speak action server. The action server takes a string and uses the Google Text-to-Speech API to generate an audio file. Then, it uses the audio output device to play the audio file.
"""

import rospy
import resampy
import tempfile
import soundfile as sf
import sounddevice as sd
from gtts import gTTS
from picker_demo.srv import Speak, SpeakRequest, SpeakResponse

AUDIO_DEVICE = 0


def text_to_speech(text: str, language: str = "en"):
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=True) as temp_file:
        # Create text to speech and save to file
        tts = gTTS(text=text, lang=language)
        tts.save(temp_file.name)

        # Read audio file
        data, sample_rate = sf.read(temp_file.name)
        rospy.logdebug(data.shape, sample_rate)

        # Resample audio if necessary
        if sample_rate != 48000:
            data = resampy.resample(data, sample_rate, 48000)

        # Play audio
        sd.play(data, 48000)
        sd.wait()


def speak(msg: SpeakRequest) -> SpeakResponse:
    """
    This function implements the speak action server. It takes a string and uses the Google Text-to-Speech API to generate an audio file. Then, it uses the audio output device to play the audio file.
    """

    # Set audio device
    sd.default.device = AUDIO_DEVICE  # type: ignore
    rospy.loginfo(f"Device: {AUDIO_DEVICE}")

    # Set language
    language = msg.language if msg.language is not None else "en"
    rospy.loginfo(f"Language: {language}")

    # Get text
    rospy.loginfo(f"Speaking: {msg.sentence}")
    text_to_speech(msg.sentence, language)

    return SpeakResponse(True)


if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.loginfo("Starting speak node...")
    rospy.init_node("speak")

    # Start the action server.
    rospy.loginfo("Starting speak action server...")
    server = rospy.Service("speak", Speak, speak)

    # Spin.
    rospy.loginfo("Speak node ready.")
    server.spin()

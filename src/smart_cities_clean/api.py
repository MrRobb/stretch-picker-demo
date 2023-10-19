#!/usr/bin/env python3

"""
This module implements the API for the demo. It is a Flask app that exposes two endpoints:
- /speak (POST): takes a JSON payload with a "sentence" field and contacts the audio server to play the sentence.
- /pick-object (POST): takes a JSON payload with a "object_class" field and contacts the robot to pick the object.

INPUT
-----
The module takes the JSON payload from the POST requests.

OUTPUT
------
The module calls a ROS service to perform the operation. The module returns a JSON payload with the data from the request.
"""

import rospy
import pyaudio
from multiprocessing import Process
from flask import Flask, abort, request, jsonify
from typing import Optional
from picker_demo.srv import Picker, PickerRequest, PickerResponse
from picker_demo.srv import Speak, SpeakRequest, SpeakResponse

app = Flask(__name__)

tts_action_client: Optional[rospy.ServiceProxy] = None
picker_action_client: Optional[rospy.ServiceProxy] = None


def tts(sentence, language):
    if tts_action_client is None:
        return
    req = SpeakRequest(sentence=sentence, language=language)
    tts_action_client.call(req)


@app.route("/speak", methods=["POST"])
def speak():
    """
    This function implements the speak endpoint. It takes a JSON payload with a "sentence" field and contacts the audio server to play the sentence.
    """
    if not request.json or not "sentence" in request.json:
        abort(400)

    if not tts_action_client:
        abort(500)

    lang_arg = request.args.get("lang")
    language = lang_arg if lang_arg is not None else "en"

    tts_action_client.wait_for_service()

    tts_service_call = Process(  # Create a daemonic process with heavy "my_func"
        target=tts, args=[request.json["sentence"], language], daemon=True
    )
    tts_service_call.start()

    return jsonify({"sentence": request.json["sentence"], "language": language})


def pick(request):
    if picker_action_client is None:
        return
    picker_action_client.call(request.json["object_class"])


@app.route("/pick-object", methods=["POST"])
def pick_object():
    """
    This function implements the pick-object endpoint. It takes a JSON payload with a "object_class" field and contacts the robot to pick the object.
    """
    if not request.json or not "object_class" in request.json:
        abort(400)

    if not picker_action_client:
        abort(500)

    picker_action_client.wait_for_service()

    picker_service_call = Process(  # Create a daemonic process with heavy "my_func"
        target=pick, args=[request], daemon=True
    )
    picker_service_call.start()

    return jsonify({"object_class": request.json["object_class"]})


@app.route("/devices", methods=["GET"])
def devices():
    # Get audio
    audio = pyaudio.PyAudio()
    count = audio.get_device_count()
    result = f"Audio Devices: {count}\n"

    # Print audio devices
    for device_index in range(count):
        device_info = audio.get_device_info_by_index(device_index)
        device_name = device_info["name"]
        result += f"{device_index} {device_name}<br>"

    return result


if __name__ == "__main__":

    # Initialize ROS node
    print("Initializing ROS node...")
    rospy.init_node("api")

    rospy.loginfo("Waiting for 'speak' action server...")
    tts_action_client = rospy.ServiceProxy("speak", Speak)

    rospy.loginfo("Waiting for 'picker' action server...")
    picker_action_client = rospy.ServiceProxy("picker", Picker)

    rospy.loginfo("Ready!")

    app.run(host="0.0.0.0", port=6000, debug=True)

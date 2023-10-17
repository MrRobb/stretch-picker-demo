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

from flask import Flask, abort, request, jsonify
from actionlib import SimpleActionClient
from std_msgs.msg import String
from typing import Optional

app = Flask(__name__)

tts_action_client: Optional[SimpleActionClient] = None
picker_action_client: Optional[SimpleActionClient] = None


@app.route("/speak", methods=["POST"])
def speak():
    """
    This function implements the speak endpoint. It takes a JSON payload with a "sentence" field and contacts the audio server to play the sentence.
    """
    if not request.json or not "sentence" in request.json:
        abort(400)

    if not tts_action_client:
        abort(500)

    sentence: String = String(request.json["sentence"])
    tts_action_client.send_goal(sentence)

    return jsonify({"sentence": request.json["sentence"]})


@app.route("/pick-object", methods=["POST"])
def pick_object():
    """
    This function implements the pick-object endpoint. It takes a JSON payload with a "object_class" field and contacts the robot to pick the object.
    """
    if not request.json or not "object_class" in request.json:
        abort(400)

    if not picker_action_client:
        abort(500)

    object_class: String = String(request.json["object_class"])
    picker_action_client.send_goal(object_class)

    return jsonify({"object_class": request.json["object_class"]})


if __name__ == "__main__":

    app.run(host="0.0.0.0", port=5000, debug=False)

    print("Waiting for 'speak' action server...")
    tts_action_client = SimpleActionClient("speak", String)
    tts_action_client.wait_for_server()

    print("Waiting for 'picker' action server...")
    picker_action_client = SimpleActionClient("picker", String)
    picker_action_client.wait_for_server()

    print("Ready!")

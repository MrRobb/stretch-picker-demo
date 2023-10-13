#!/usr/bin/env python3

import time
import pyaudio
import tempfile
import sounddevice as sd
import soundfile as sf
import resampy
import stretch_body.robot
from stretch_body.hello_utils import deg_to_rad
from flask import Flask, request
from gtts import gTTS

# Audio device
AUDIO_DEVICE = 0

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

# Web app
app = Flask(__name__)

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


def check_sample_rate():
    samplerates = range(0, 130000, 1000)

    supported_samplerates = []
    for fs in samplerates:
        try:
            sd.check_output_settings(device=AUDIO_DEVICE, samplerate=fs)
        except Exception:
            print("Exception")
        else:
            supported_samplerates.append(fs)
    print("Supported sample rates:", supported_samplerates)


def list_audio_devices() -> str:
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


def text_to_speech(text: str, language: str = "en"):
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=True) as temp_file:
        # Create text to speech and save to file
        tts = gTTS(text=text, lang=language)
        tts.save(temp_file.name)

        # Read audio file
        data, sample_rate = sf.read(temp_file.name)
        print(data.shape, sample_rate)

        # Resample audio if necessary
        if sample_rate != 48000:
            data = resampy.resample(data, sample_rate, 48000)

        # Play audio
        sd.play(data, 48000)
        sd.wait()


@app.route("/pick-object", methods=["POST"])
def speak():
    # Set audio device
    device = int(request.args.get("device")) if request.args.get("device") is not None else AUDIO_DEVICE  # type: ignore
    sd.default.device = device  # type: ignore
    print("Device:", device)

    # Set language
    lang_arg = request.args.get("lang")
    language = lang_arg if lang_arg is not None else "en"
    print("Language:", language)

    # Get text
    text = request.get_json()
    print("Text:", text)
    if (
        text is not None
        and "additional_parameters" in text
        and text["additional_parameters"] is not None
        and "object_class" in text["additional_parameters"]
    ):
        text_to_speech(text["additional_parameters"]["object_class"], language)
    return text


print("Resetting arm...")
reset_arm()
print("Resetting arm... ✅")

print("Looking at arm...")
look_at_arm()
print("Looking at arm... ✅")

# retract_extend()
# doubt_what_to_pick()

# robot_dance()

print("Lifting arm...")
# for _ in range(4):
#     reset_arm()
#     robot_lift("bottle")
print("Lifting arm... ✅")

# Move base to the right
# r.base.translate_by(0.1) # 10 cm
# r.push_command()
# r.base.wait_until_at_setpoint()

# Move base to the right 1cm
# r.base.translate_by(0.01)
# r.push_command()
# r.base.wait_until_at_setpoint()

# Extend arm
r.arm.move_to(0.5)
r.push_command()
r.arm.wait_until_at_setpoint()

# look_ahead()
# reset_arm()

r.stop()

# app.run(host="0.0.0.0", port=6000, debug=False)

#!/usr/bin/env python3

import pyaudio
import tempfile
import sounddevice as sd
import soundfile as sf
import resampy
from flask import Flask, request
from gtts import gTTS

AUDIO_DEVICE = 0

# Web app
app = Flask(__name__)


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


@app.route("/speak", methods=["POST"])
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
    if text is not None and "sentence" in text and text["sentence"] is not None:
        print("Text:", text["sentence"])
        text_to_speech(text["sentence"], language)
    else:
        print("Text is None")
    return text


@app.route("/devices", methods=["GET"])
def devices():
    return list_audio_devices()


if __name__ == "__main__":

    app.run(host="0.0.0.0", port=5000, debug=False)

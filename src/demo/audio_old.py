#!/usr/bin/env python3

import pyaudio
import tempfile
import sounddevice as sd
import soundfile as sf
import resampy
from gtts import gTTS

AUDIO_DEVICE = 0

# Set audio device
sd.default.device = AUDIO_DEVICE  # type: ignore


def check_sample_rate():
    samplerates = range(0, 130000, 1000)

    supported_samplerates = []
    for fs in samplerates:
        try:
            sd.check_output_settings(device=AUDIO_DEVICE, samplerate=fs)
        except Exception:
            pass
        else:
            supported_samplerates.append(fs)
    print(supported_samplerates)


def print_audio_devices():
    # Get audio
    audio = pyaudio.PyAudio()
    count = audio.get_device_count()
    print(f"Audio Devices: {count}")

    # Print audio devices
    for device_index in range(count):
        device_info = audio.get_device_info_by_index(device_index)
        print(device_index, device_info)


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


# Print devices
print_audio_devices()

# Speak response
text_to_speech("Hello, my name is Stretch. I am a mobile manipulation robot.")

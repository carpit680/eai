#!/usr/bin/env python3

from gtts import gTTS
from pydub import AudioSegment
import numpy as np
import io
import time
import simpleaudio as sa
from concurrent.futures import ThreadPoolExecutor

def text_to_audio_segment(text, speed):
    tts = gTTS(text=text, lang='en', slow=False)
    fp = io.BytesIO()
    tts.write_to_fp(fp)
    fp.seek(0)
    audio_segment = AudioSegment.from_file(fp, format="mp3")
    if speed != 1.0:
        audio_segment = audio_segment.speedup(playback_speed=speed)
    return audio_segment

def concatenate_audio_segments(segments):
    combined = AudioSegment.empty()
    for segment in segments:
        combined += segment
    return combined

def generate_speech(text, speed=1.0):
    # Split the text into chunks to avoid long processing times and handle large texts
    chunk_size = 500  # Adjust chunk size as needed
    text_chunks = [text[i:i+chunk_size] for i in range(0, len(text), chunk_size)]
    
    with ThreadPoolExecutor() as executor:
        # Generate audio segments in parallel
        futures = [executor.submit(text_to_audio_segment, chunk, speed) for chunk in text_chunks]
        audio_segments = [future.result() for future in futures]
    
    # Concatenate all audio segments
    combined_audio = concatenate_audio_segments(audio_segments)
    
    # Convert to raw audio data
    raw_data = np.array(combined_audio.get_array_of_samples())
    sample_rate = combined_audio.frame_rate
    num_channels = combined_audio.channels
    
    # Create wave object from raw audio data
    wave_obj = sa.WaveObject(raw_data.tobytes(), num_channels, combined_audio.sample_width, sample_rate)
    return wave_obj

def text_to_speech(text, speed=1.0):
    """
    Converts the given text to speech and plays it.

    Parameters:
    text (str): The text to convert to speech.
    speed (float): The speed at which to play the audio (1.0 is normal speed).
    """
    try:
        start_time = time.time_ns()
        wave_obj = generate_speech(text, speed)
        print(f"time taken: {(time.time_ns() - start_time)/1e6} ms")

        # Play the audio using simpleaudio (blocking)
        play_obj = wave_obj.play()
        play_obj.wait_done()

    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
if __name__ == "_main_":
    text = "Hello, this is a test of the text to speech conversion with larger texts." * 10
    text_to_speech(text, speed=1.3)
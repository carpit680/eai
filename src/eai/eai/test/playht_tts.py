from pydub import AudioSegment
from pydub.playback import play
from pyht import Client, TTSOptions, Format
from io import BytesIO
import os


class PlayHTTTS:
    def __init__(self, api_key=None, api_secret=None):
        # get api key and api secret from env variables from the system
        if api_key is None or api_secret is None:
            api_key = os.environ.get("PLAY_HT_API_KEY")
            api_secret = os.environ.get("PLAY_HT_API_SECRET")
        
        self.client = Client(api_key, api_secret)

    def generate_and_play_audio(self, text, speed=1.0):
        # Configure options for TTS
        options = TTSOptions(
            voice="s3://voice-cloning-zero-shot/d9ff78ba-d016-47f6-b0ef-dd630f59414e/female-cs/manifest.json",
            sample_rate=44_100,
            format=Format.FORMAT_MP3,
            speed=0.8,
        )

        # Start streaming!
        audio_data = b""
        for chunk in self.client.tts(text=text, voice_engine="PlayHT2.0-turbo", options=options):
            audio_data += chunk

        # Load the audio data into an AudioSegment
        try:
            audio = AudioSegment.from_mp3(BytesIO(audio_data))

            # Play the audio
            play(audio)
        except:
            pass

if __name__ == "__main__":
    playht_tts = PlayHTTTS()
    text = "Hello, this is a test of the text to speech conversion."
    playht_tts.generate_and_play_audio(text, speed=0.8)
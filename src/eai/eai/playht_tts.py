from pydub import AudioSegment
from pydub.playback import play
from pyht import Client, TTSOptions, Format
from io import BytesIO

# Initialize the PlayHTTTS object with your API credentials
api_key = "vwGo6SD9f5WyraNyjFEnukXljgv1"
api_secret = "ececd4cc81f14c1c94650142035a2de1"

class PlayHTTTS:
    def __init__(self, api_key = api_key, api_secret = api_secret):
        self.client = Client(api_key, api_secret)

    def generate_and_play_audio(self, text):
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

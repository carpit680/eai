from gtts import gTTS
from pydub import AudioSegment
from pydub.playback import play
import io
import simpleaudio as sa

def text_to_speech(text):
    """
    Converts the given text to speech and plays it.

    Parameters:
    text (str): The text to convert to speech.
    """
    try:
        # Initialize gTTS object with the provided text and language
        tts = gTTS(text=text, lang='en', slow=False)

        # Save the speech to an in-memory file
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        # Load the audio file from the in-memory file
        audio = AudioSegment.from_file(fp, format="mp3")
        
        # Export the audio to a wav file (simpleaudio does not support mp3)
        wav_fp = io.BytesIO()
        audio.export(wav_fp, format="wav")
        wav_fp.seek(0)
        
        # Load the wav audio file into simpleaudio
        wave_obj = sa.WaveObject.from_wave_file(wav_fp)
        
        # Play the audio using simpleaudio (blocking)
        play_obj = wave_obj.play()
        
        # Wait for playback to finish
        play_obj.wait_done()

    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
if __name__ == "__main__":
    text = "Hello, this is a test of the text to speech conversion."
    text_to_speech(text)

#!/usr/bin/env python3

import sys
import time
import json
import queue
import sounddevice as sd
from threading import Lock
from llama import llama3_groq
from playht_tts import PlayHTTTS
from vosk import Model, KaldiRecognizer
from text_to_speech import text_to_speech
from kosmos2 import kosmos2

kosmos = kosmos2(False)
q = queue.Queue()

llm = llama3_groq(False)
try:
    playht_tts = PlayHTTTS()
    tts = playht_tts.generate_and_play_audio
except:
    print("PlayHTTTS out of credits. Using text_to_speech.")
    tts = text_to_speech

listening_lock = Lock()
is_speaking = False

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    global is_speaking
    if status:
        print(status, file=sys.stderr)
    if not is_speaking:
        q.put(bytes(indata))

device = None

try:
    # Query device information
    device_info = sd.query_devices(device, "input")
    samplerate = int(device_info["default_samplerate"])
except Exception as e:
    print(f"Error querying devices: {e}", file=sys.stderr)
    sys.exit(1)

try:
    # Load the Vosk model
    model = Model(lang="en-us")
except Exception as e:
    print(f"Error loading Vosk model: {e}", file=sys.stderr)
    sys.exit(1)

# Create a Kaldi recognizer with the model and sample rate
rec = KaldiRecognizer(model, samplerate)

# Variable to track if we are currently listening
is_listening = False
# Variable to track the time of the last speech activity
last_speech_time = time.time()
# Threshold for the duration of silence (5 seconds)
silence_threshold = 0.5
just_responded = False
name = "travis"
responding = False
conversation_history = []
print("#"*100)

try:
    with sd.RawInputStream(
            samplerate=samplerate,
            blocksize=8000,
            device=device,
            dtype="int16",
            channels=1,
            callback=callback
        ):

        print("Listening for the keyword")

        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get('text', '')
                
                if not is_listening and name in text.lower():
                    print(f"Detected with Keyword: {text}")
                    is_listening = True
                    last_speech_time = time.time()
                    if is_listening and "stop" in text.lower():
                        is_listening = False

                if is_listening:
                    if text.strip():
                        last_speech_time = time.time()

            else:
                partial_result = json.loads(rec.PartialResult())
                partial_text = partial_result.get('partial', '')

                if is_listening:
                    if partial_text.strip():
                        # print("partial_text" + partial_text)
                        last_speech_time = time.time()
                    elif text.strip():
                        if time.time() - last_speech_time > silence_threshold:
                            
                            print(f"REQUEST: {text}")
                            prompt = f"""
You are a personal virtual AI assistant named {name}. Reply in brief to the following question/instruction
NOTE: Answer in a conversational tone so that it can smoothly be converted to speech. Do not use any symbols or special characters that would be awkward in a conversation. Use your response history given below for context if required and available.

NOTE: If your response to the question/instruction/Statement from the user requires a response from the user or your response is a question then add the last keyword to the response as "True" else "False" in the next line.

Response history:
{str(conversation_history)}

Question/Instruction/Statement: {text}

Answer:
"""
                            prompt_env = f"""
Return "True" or "False" as the only response and nothing else based on if the question asked by the user is anyway related to your immediate environment, your surroundings, what you see, or about an object that might be in your view.

Example1:
    Question/Instruction/Statement: What is the color of the table?
    Response: True

Example2:
    Question/Instruction/Statement: What do you see?
    Response: True

Example3:
    Question/Instruction/Statement: How is the weather today?
    Response: False

Question/Instruction/Statement: {text}
Response:
"""

                            response_env = llm.get_response(prompt_env, False)
                            print(f"ENVIRONMENT: {response_env}")
                            if response_env.lower() == "true":

                            response, conversation_history = llm.get_response(prompt, True)
                            response_req = response.split('\n')[-1]
                            filtered_response = response.split('\n')[:-1]
                            filtered_response = ' '.join(filtered_response)
                            print(f"RESPONSE: {filtered_response}")
                            is_speaking = True
                            if not filtered_response.strip():
                                filtered_response = "Sorry, I didn't get that. Can you please try again?"
                                response_req = "true"
                            # text_to_speech(filtered_response)
                            tts(filtered_response, 1.3)
                            print("#"*100)
                            last_speech_time = time.time()
                            # Clear the flag after speech playback is complete
                            is_speaking = False
                            is_listening = False
                            if response_req.lower() == "true":
                                # time.sleep(0.6)
                                is_listening = True
                                text = ""
                                print("Listening again...")

except KeyboardInterrupt:
    print("\nDone")
except Exception as e:
    print(f"Error during recording or recognition: {e}", file=sys.stderr)
    sys.exit(1)

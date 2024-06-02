#!/usr/bin/env python3
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from llama import llama3_groq
from text_to_speech import text_to_speech
import os
import time
import json

q = queue.Queue()

llm = llama3_groq(True)
def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
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
silence_threshold = 2
just_responded = False
name = "travis"
responding = False
conversation_history = []
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
            if not responding:
                data = q.get()
            else: continue
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get('text', '')
                
                if not is_listening and name in text.lower():
                    print("Keyword detected. Listening...")
                    print(f"detected with kw {text}")
                    is_listening = True
                    last_speech_time = time.time()

                if is_listening:
                    print("still listening")
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
                            
                            print(text)
                            prompt = f"""
You are a personal virtual AI assistant named {name}. Reply in brief to the following question/instruction
NOTE: Answer in a conversational tone so that it can smoothly be converted to speech. Do not use any symbols or special characters that would be awkward in a conversation. Use your response history given below for context if required and available.

NOTE: If your response to the question/instruction/Statement from the user requires a response from the user or your response is a question then add the last keyword to the response as "True" else "False" in the next line.

Response history:
{str(conversation_history)}

Question/Instruction/Statement: {text}

Answer:
"""
                            responding = True
                            response, conversation_history = llm.get_response(prompt)
                            # get last word after the last new line
                            response_req = response.split('\n')[-1]
                            print(response_req)
                            filtered_response = response.split('\n')[:-1]
                            # convert filtered response to string
                            filtered_response = ' '.join(filtered_response)
                            is_listening = False
                            text_to_speech(filtered_response)
                            last_speech_time = time.time()
                            print("response complete ##############################")
                            # convert reponse_req to True or false
                            if response_req.lower() == "true":
                                response_req = True
                            else:
                                response_req = False
                            is_listening = response_req
                            responding = False
                            text  = ""

except KeyboardInterrupt:
    print("\nDone")
except Exception as e:
    print(f"Error during recording or recognition: {e}", file=sys.stderr)
    sys.exit(1)

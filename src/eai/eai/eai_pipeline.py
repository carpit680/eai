#!/usr/bin/env python3

import sys
import time
import json
import queue
import sounddevice as sd
from rclpy.node import Node
from threading import Lock
from eai.llama import llama3_groq
from eai.playht_tts import PlayHTTTS
from vosk import Model, KaldiRecognizer
from eai.text_to_speech import text_to_speech
from eai.kosmos2 import kosmos2
from cv_bridge import CvBridge
from collections import deque
from sensor_msgs.msg import Image
from eai.image_subscriber import ImageSubscriber
from threading import Thread


class EaiPipeline(Node):
    def __init__(self, agent_name="travis"):
        super().__init__('eai_pipeline')
        self.image_subscriber = ImageSubscriber()

        self.q = queue.Queue()
        self.listening_lock = Lock()
        self.kosmos = kosmos2(False)
        self.llm = llama3_groq(False)

        self.name = agent_name
        self.is_speaking = False
        self.device = None
        self.listening_thread = Thread(target=self.start_listening)
        try:
            model = Model(lang="en-us")
        except Exception as e:
            print(f"Error loading Vosk model: {e}", file=sys.stderr)
            sys.exit(1)

        try:
            device_info = sd.query_devices(self.device, "input")
            self.samplerate = int(device_info["default_samplerate"])
        except Exception as e:
            print(f"Error querying devices: {e}", file=sys.stderr)
            sys.exit(1)

        self.rec = KaldiRecognizer(model, self.samplerate)

        try:
            playht_tts = PlayHTTTS()
            self.tts = playht_tts.generate_and_play_audio
        except:
            print("PlayHTTTS out of credits. Using text_to_speech.")
            self.tts = text_to_speech

        self.listening_thread.start()


    def callback(self, indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        if not self.is_speaking:
            self.q.put(bytes(indata))


    def start_listening(self):
        is_listening = False
        last_speech_time = time.time()
        silence_threshold = 0.5
        conversation_history= []
        env_description = ''
        text = ''
        print("#"*100)

        try:
            with sd.RawInputStream(
                    samplerate=self.samplerate,
                    blocksize=8000,
                    device=self.device,
                    dtype="int16",
                    channels=1,
                    callback=self.callback
                ):

                print("Listening for the keyword")

                while True:
                    data = self.q.get()
                    if self.rec.AcceptWaveform(data):
                        result = json.loads(self.rec.Result())
                        text = result.get('text', '')
                        if not is_listening and self.name in text.lower():
                            print("Detected keyword!")
                            last_speech_time = time.time()
                            is_listening = True
                            if "stop" in text.lower():
                                is_listening = False

                        if is_listening and text.strip():
                            last_speech_time = time.time()

                    else:
                        # text = input() # uncomment for textual input
                        # is_listening = True # uncomment for textual input
                        partial_result = json.loads(self.rec.PartialResult())
                        partial_text = partial_result.get('partial', '')

                        if is_listening:
                            if partial_text.strip():
                                # print("partial_text" + partial_text)
                                last_speech_time = time.time()
                            elif text.strip():
                                if time.time() - last_speech_time > silence_threshold:
                                    print(f"REQUEST: {text}")
                                    # NOTE: You can see around in your environment using a Vision Language Model (VLM). If you have any queries about the environment or your surroundings or what you see or about an object that might be in your view then respond with your query.
                                    prompt_env = f"""
INSTRUCTION: You can see around in your environment using a Vision Language Model (VLM). Based on if the question asked by the user is anyway related to your immediate environment, your surroundings, what you see, or about an object that might be in your view answer with a question you would like to ask the VLM. Phrase your question as a prompt to the VLM. Frame the question as if you are asking about an image.

Example1:
Question/Instruction/Statement: what is the color of the table
Response: What is the color of the table?

Example2:
Question/Instruction/Statement: what do you see
Response: <grounding> Describe every object in detail with their relative locations in the environment: 

Example3:
Question/Instruction/Statement: what should I wear today
Response: False

Example4:
Question/Instruction/Statement: what color is the pot
Response: What is the color of the pot in the image?

Example5:
Question/Instruction/Statement: where is the plant
Response: <grounding> Where is the plant placed relative to other objects in the room?

Question/Instruction/Statement: {text}
Response:
"""

                                    response_env = self.llm.get_response(prompt_env, False)
                                    # print(f"response_env: {response_env}")
                                    if response_env.lower() != "false":
                                        self.get_logger().info(f"VLM REQUEST: {response_env}")
                                        env_image = self.image_subscriber.get_latest_frame()
                                        env_description, entities, captioned_image = self.kosmos.ground_frame(env_image, prompt=response_env)
                                        self.get_logger().info(f"VLM RESPONSE: {env_description}")
                                        # print(f"env_description: {env_description}")
                                    
                                    prompt = f"""
    You are a personal virtual AI assistant named {self.name}. Reply in brief to the following question/instruction

    NOTE: Answer in a conversational tone so that it can smoothly be converted to speech. Do not use any symbols or special characters that would be awkward in a conversation. Use your response history given below for context if required and available.

    NOTE: If your response to the question/instruction/Statement from the user requires a response from the user or your response is a question then add the last keyword to the response as "True" else "False" in the next line.

    NOTE: You can see around in your environement using a Vision Language Model. Use information provided in "Environment Description" below if needed to answer any question related to your surroundings or what you see or objects in your view. Do not add anything that does not exist in the environment descriptionn to your response.

    Environment Description: {env_description}

    Response history: {str(conversation_history)}

    Question/Instruction/Statement: {text}

    Answer:
    """
                                    # print(f"prompt: {prompt}")
                                    response, conversation_history= self.llm.get_response(prompt, True)
                                    response_req = response.split('\n')[-1]
                                    filtered_response = response.split('\n')[:-1]
                                    filtered_response = ' '.join(filtered_response)
                                    self.is_speaking = True
                                    if not filtered_response.strip():
                                        # print("filtered_response" + filtered_response)
                                        filtered_response = "Sorry, I didn't get that. Can you please try again?"
                                        response_req = "true"
                                    print(f"RESPONSE: {filtered_response}")
                                    self.tts(filtered_response, 1.3)
                                    print("#"*100)
                                    last_speech_time = time.time()
                                    # Clear the flag after speech playback is complete
                                    self.is_speaking = False
                                    is_listening = False
                                    if response_req.lower() == "true":
                                        is_listening = True
                                        text = ""
                                        print("Listening again...")

        except KeyboardInterrupt:
            print("\nDone")
        # except Exception as e:
        #     print(f"Error during recording or recognition: {e}", file=sys.stderr)
        #     sys.exit(1)

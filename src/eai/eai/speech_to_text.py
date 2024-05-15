#!/usr/bin/env python3
import queue
import sys
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import os

q = queue.Queue()

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

device = None
device_info = sd.query_devices(device, "input")
samplerate = int(device_info["default_samplerate"])
    
model = Model(lang="en-us")

with sd.RawInputStream(
        samplerate=samplerate,
        blocksize = 8000,
        device=device,
        dtype="int16",
        channels=1,
        callback=callback
    ):

    rec = KaldiRecognizer(model, samplerate)
    while True:
        try:
            data = q.get()
            if rec.AcceptWaveform(data):
                print(rec.Result())
            else:
                print(rec.PartialResult())

        except KeyboardInterrupt:
            print("\nDone")
            break

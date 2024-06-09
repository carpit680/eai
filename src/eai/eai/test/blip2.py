#!/usr/bin/env python3

import json
import torch
import requests
from PIL import Image
from transformers import BlipProcessor, BlipForConditionalGeneration, AutoConfig

MODEL_PATH = "Salesforce/blip-image-captioning-large"

with open('/home/arpit/eai/eai/src/eai/eai/test/blip2-config.json', 'r') as file:
    config_dict = json.load(file)


config = AutoConfig.from_pretrained(MODEL_PATH,**config_dict)

processor = BlipProcessor.from_pretrained(MODEL_PATH)
model = BlipForConditionalGeneration.from_pretrained(MODEL_PATH, config=config, torch_dtype=torch.float16).to("cuda")

img_url = 'https://storage.googleapis.com/sfr-vision-language-research/BLIP/demo.jpg' 
raw_image = Image.open(requests.get(img_url, stream=True).raw).convert('RGB')

# conditional image captioning
text = "a photography of"
inputs = processor(raw_image, text, return_tensors="pt").to("cuda", torch.float16)

out = model.generate(**inputs)
print(processor.decode(out[0], skip_special_tokens=True))
# >>> a photography of a woman and her dog

# unconditional image captioning
inputs = processor(raw_image, return_tensors="pt").to("cuda", torch.float16)

out = model.generate(**inputs)
print(processor.decode(out[0], skip_special_tokens=True))
# >>> a woman sitting on the beach with her dog

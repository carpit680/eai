import requests
import json
import torch
import cv2
from PIL import Image as PILImage
from transformers import AutoProcessor, AutoModelForVision2Seq, AutoConfig


class kosmos2():
    def __init__(self, get_logger):
        model_path = "microsoft/kosmos-2-patch14-224"
        config_path = "/home/arpit/eai/eai/src/eai/eai/test/kosmos/kosmos2-config.json"
        self.prompt = "<grounding> Describe the image and object placement in detail"

        with open(config_path, 'r') as file:
            config_dict = json.load(file)

        config = AutoConfig.from_pretrained(model_path,**config_dict)
        self.get_logger = get_logger
        self.model = AutoModelForVision2Seq.from_pretrained(model_path, config=config, device_map="cuda", torch_dtype=torch.float16)
        self.processor = AutoProcessor.from_pretrained(model_path)

        self.get_logger().info("Kosmos2 initialized.")

    def ground_frame(self, image):
        pil_image = PILImage.fromarray(image)
        inputs = self.processor(text=self.prompt, images=pil_image, return_tensors="pt").to(self.model.device)

        generated_ids = self.model.generate(
            pixel_values=inputs["pixel_values"],
            input_ids=inputs["input_ids"],
            attention_mask=inputs["attention_mask"],
            image_embeds=None,
            image_embeds_position_mask=inputs["image_embeds_position_mask"],
            use_cache=True,
            max_new_tokens=512,
        )
        generated_text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

        processed_text, entities = self.processor.post_process_generation(generated_text)
        # draw the bounding box in the entities variable on the image and write the names on the boxes
        for entity in entities:
            name = entity[0]
            

        # `[('a snowman', (12, 21), [(0.390625, 0.046875, 0.984375, 0.828125)]), ('a fire', (41, 47), [(0.171875, 0.015625, 0.484375, 0.890625)])]`
        return processed_text, entities, image

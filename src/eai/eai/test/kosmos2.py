import json
import torch
import cv2
import random
from PIL import Image as PILImage
from transformers import AutoProcessor, AutoModelForVision2Seq, AutoConfig

def random_color():
    """
    Returns a random color in RGB format.
    """
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return (r, g, b)

class kosmos2():
    def __init__(self, debug):
        model_path = "microsoft/kosmos-2-patch14-224"
        config_path = "/home/arpit/eai/eai/src/eai/eai/test/kosmos2-config.json"
        self.prompt = "<grounding> Describe every object in detail with their relative locations in the environment: "
        
        with open(config_path, 'r') as file:
            config_dict = json.load(file)

        self.debug = debug

        config = AutoConfig.from_pretrained(model_path,**config_dict)
        self.model = AutoModelForVision2Seq.from_pretrained(model_path, config=config, device_map="cuda", torch_dtype=torch.float16)
        self.processor = AutoProcessor.from_pretrained(model_path)

        print("Kosmos2 initialized.")

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
            max_new_tokens=1024,
        )
        generated_text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

        processed_text, entities = self.processor.post_process_generation(generated_text)
        processed_text = processed_text[len(self.prompt)-12:]

        if self.debug:
            for entity in entities:
                color = random_color()
                name = entity[0]
                text_ctr = entity[1]
                bbox = entity[2][0]
                x1, y1, x2, y2 = bbox
                x1 = int(x1 * image.shape[1])
                y1 = int(y1 * image.shape[0])
                x2 = int(x2 * image.shape[1])
                y2 = int(y2 * image.shape[0])
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(image, name, (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            print(processed_text)

        return processed_text, entities, image
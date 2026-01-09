from google import genai
from google.genai import types

import os 
from dotenv import load_dotenv

from prompts import ASSIGN_ROOM_LABEL_ONLY_PROMPT

import numpy as np
from PIL import Image
import io

from vlm_output_schema import UserSemanticTarget

class VLMClient():
    def __init__(self, model_id="gemini-robotics-er-1.5-preview"):
        # Load API Key from .env
        load_dotenv()
        GEMINI_API_KEY = os.getenv('GEMINI_API_KEY')

        self.model_options = ["gemini-robotics-er-1.5-preview", "gemini-2.5-flash"]
        self.model_id = model_id
        # self.model_id = "gemini-2.5-flash"
        self.client = genai.Client(api_key=GEMINI_API_KEY)

    def image_text_query(self, image_prompt : np.ndarray, text_prompt : str):

        # Convert the NumPy array to a PIL Image object
        pil_image = Image.fromarray(image_prompt)

        # Create an in-memory binary stream
        byte_arr = io.BytesIO()

        # Save the PIL Image to the stream in PNG format
        pil_image.save(byte_arr, format='PNG')

        # Get the bytes object
        image_bytes = byte_arr.getvalue()

        # TODO: THIS IS BAD FIX THIS
        try:
            image_response = self.client.models.generate_content(
                model=self.model_id,
                contents=[
                    types.Part.from_bytes(
                        data=image_bytes,
                        mime_type='image/png',
                    ),
                    text_prompt
                ],
                config = types.GenerateContentConfig(
                    temperature=0.5,
                    thinking_config=types.ThinkingConfig(thinking_budget=0)
                )
            )
        except:
            image_response = None # TODO: Understand what to do here
        return image_response

    def text_query(self, text_prompt : str):
        text_response = self.client.models.generate_content(
            model=self.model_id,
            contents=[
                text_prompt
            ],
            config = types.GenerateContentConfig(
                response_mime_type = "application/json",
                temperature=0.5,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
                response_json_schema=UserSemanticTarget.model_json_schema()
            )
        )
        return text_response
    
    def text_query_output_schema(self, text_prompt : str, schema):
        text_response = self.client.models.generate_content(
            model=self.model_id,
            contents=[
                text_prompt
            ],
            config = types.GenerateContentConfig(
                response_mime_type = "application/json",
                temperature=0.5,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
                response_json_schema=schema
            )
        )
        return text_response

if __name__ == '__main__':
    vlm_client = VLMClient()

    # TODO: Change this file path
    img = Image.open('/Users/pravaltelagi/oakd_camera/saves/apartment/rgb_imgs/0008.png')
    img = np.asarray(img)
    response = vlm_client.image_text_query(img, ASSIGN_ROOM_LABEL_ONLY_PROMPT)
    print(response.text)


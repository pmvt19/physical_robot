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

        self.model_options = ["gemini-robotics-er-1.5-preview", "gemini-2.5-flash", "gemini-2.5-pro", "gemini-2.0-flash", "gemini-2.0-flash-lite"]
        self.current_model_options_idx = 0
        self.model_id = model_id
        self.client = genai.Client(api_key=GEMINI_API_KEY)

    def _get_model_id(self):
        new_model_id = self.model_options[self.current_model_options_idx % len(self.model_options)]
        print(f"Using Model: {new_model_id}")
        return new_model_id
    
    def protect_failed_api_calls(base_fn):
        def enhanced_fn(*args, **kwargs):
            while True:
                try:
                    return base_fn(*args, **kwargs)
                except:
                    user_input = input("Call to VLM Returned an Exception. Try Again? Or Swap Models? Or Quit?")
                    if user_input.lower() == 'try again':
                        continue
                    elif user_input.lower() == 'swap models':
                        args[0].current_model_options_idx += 1
                    else:
                        print("Returning `None`")
                        break
            return None
        return enhanced_fn

    def _get_image_bytes(self, image: np.ndarray):
        # Convert the NumPy array to a PIL Image object
        pil_image = Image.fromarray(image)

        # Create an in-memory binary stream
        byte_arr = io.BytesIO()

        # Save the PIL Image to the stream in PNG format
        pil_image.save(byte_arr, format='PNG')

        # Get the bytes object
        image_bytes = byte_arr.getvalue()

        return image_bytes

    def _image_text_query_schema(self, image_prompt: np.ndarray, text_prompt: str, schema: dict):
        
        image_response = self.client.models.generate_content(
            model=self._get_model_id(),
            contents=[
                types.Part.from_bytes(
                    data=self._get_image_bytes(image_prompt),
                    mime_type='image/png',
                ),
                text_prompt
            ],
            config = types.GenerateContentConfig(
                response_mime_type="application/json",
                temperature=0.5,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
                response_json_schema=schema
            )
        )
        return image_response
    
    def _image_text_query_no_schema(self, image_prompt: np.ndarray, text_prompt: str):

        image_response = self.client.models.generate_content(
            model=self._get_model_id(),
            contents=[
                types.Part.from_bytes(
                    data=self._get_image_bytes(image_prompt),
                    mime_type='image/png',
                ),
                text_prompt
            ],
            config = types.GenerateContentConfig(
                temperature=0.5,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
            )
        )
        return image_response
    
    @protect_failed_api_calls
    def image_text_query(self, image_prompt: np.ndarray, text_prompt: str, schema: dict = None):
        if schema is not None:
            # Query Using Schema
            return self._image_text_query_schema(image_prompt=image_prompt, text_prompt=text_prompt, schema=schema)
        else:
            # Query Using No Schema
            return self._image_text_query_no_schema(image_prompt=image_prompt, text_prompt=text_prompt)
    
    def _text_query_schmea(self, text_prompt: str, schema: dict):
        text_response = self.client.models.generate_content(
            model=self._get_model_id(),
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
    
    def _text_query_no_schmea(self, text_prompt: str):
        text_response = self.client.models.generate_content(
            model=self._get_model_id(),
            contents=[
                text_prompt
            ],
            config = types.GenerateContentConfig(
                temperature=0.5,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
            )
        )
        return text_response
    
    @protect_failed_api_calls
    def text_query(self, text_prompt: str, schema: dict = None):
        if schema is not None:
            # Query Using Schema
            return self._text_query_schmea(text_prompt=text_prompt, schema=schema)
        else:
            # Query Using No Schema
            return self._text_query_no_schmea(text_prompt=text_prompt)

if __name__ == '__main__':
    vlm_client = VLMClient()

    # TODO: Change this file path
    img = Image.open('/Users/pravaltelagi/oakd_camera/saves/apartment/rgb_imgs/0004.png')
    img = np.asarray(img)
    response = vlm_client.image_text_query(img, ASSIGN_ROOM_LABEL_ONLY_PROMPT)
    print(response.text)


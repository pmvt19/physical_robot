from ollama import chat
from prompts import EXTRACT_SEMANTIC_TARGETS, ASSIGN_ROOM_LABEL_ONLY_PROMPT
from vlm_output_schema import UserSemanticTarget
import numpy as np
from PIL import Image
import io

class GemmaVLMClient():
    def __init__(self):
        self.model = 'gemma3:4b'

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
        response = chat(
            model=self.model,
            messages=[
                {
                    'role': 'user', 
                    'content': text_prompt,
                    "images": [self._get_image_bytes(image_prompt)]
                }],
            format=schema
        )
        return response.message.content
    
    def _image_text_query_no_schema(self, image_prompt: np.ndarray, text_prompt: str):
        response = chat(
            model=self.model,
            messages=[
                {
                    'role': 'user', 
                    'content': text_prompt,
                    "images": [self._get_image_bytes(image_prompt)]
                }]
        )
        return response.message.content
    
    def image_text_query(self, image_prompt: np.ndarray, text_prompt: str, schema: dict = None):
        if schema is not None:
            # Query Using Schema
            return self._image_text_query_schema(image_prompt=image_prompt, text_prompt=text_prompt, schema=schema)
        else:
            # Query Using No Schema
            return self._image_text_query_no_schema(image_prompt=image_prompt, text_prompt=text_prompt)
    
    def _text_query_schmea(self, text_prompt: str, schema: dict):
        response = chat(
            model=self.model,
            messages=[
                {
                    'role': 'user', 
                    'content': text_prompt
                }],
            format=schema
        )
        return response.message.content
    
    def _text_query_no_schmea(self, text_prompt: str):
        response = chat(
            model=self.model,
            messages=[
                {
                    'role': 'user', 
                    'content': text_prompt
                }]
        )
        return response.message.content
    
    def text_query(self, text_prompt: str, schema: dict = None):
        if schema is not None:
            # Query Using Schema
            return self._text_query_schmea(text_prompt=text_prompt, schema=schema)
        else:
            # Query Using No Schema
            return self._text_query_no_schmea(text_prompt=text_prompt)

if __name__ == '__main__':
    vlm_client = GemmaVLMClient()

    user_input = input("Please provide where you want the robot to travel (object or room)\n")

    text_prompt = EXTRACT_SEMANTIC_TARGETS.format(user_input, 
                                                ['kitchen', 'bedroom', 'office'], # Valid Rooms
                                                ['oven', 'refridgerator', 'stove', 'desk', 'chair', 'bed'], # Valid Objects
                                                ['person', 'wall']) # Invalid Objects
    
    print(text_prompt)

    vlm_response = vlm_client.text_query(text_prompt, UserSemanticTarget.model_json_schema())
    user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)
    print(user_semantic_target)

    img = Image.open(r"C:\Users\Praval Telagi\Pictures\kitchen.jpg")
    img = np.asarray(img)
    response = vlm_client.image_text_query(img, ASSIGN_ROOM_LABEL_ONLY_PROMPT)
    print(response)

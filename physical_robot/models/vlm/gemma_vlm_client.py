from ollama import chat
from prompts import EXTRACT_SEMANTIC_TARGETS, ASSIGN_ROOM_LABEL_ONLY_PROMPT
from vlm_output_schema import UserSemanticTarget
import numpy as np
from PIL import Image
import io

from physical_robot.models.vlm.abstract_vlm_client import AbstractVLMClient

class GemmaVLMClient(AbstractVLMClient):
    def __init__(self):
        self.model = 'gemma3:4b'

        print("To use the GemmaVLMClient. Ensure there is a local Ollama server running with the Gemma3:4b model downloaded")

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

if __name__ == '__main__':
    vlm_client = GemmaVLMClient()

    user_input = input("Please provide where you want the robot to travel (object or room)\n")

    text_prompt = EXTRACT_SEMANTIC_TARGETS.format(user_input, 
                                                ['kitchen', 'bedroom', 'office'], # Valid Rooms
                                                ['oven', 'refridgerator', 'stove', 'desk', 'chair', 'bed'], # Valid Objects
                                                ['person', 'wall']) # Invalid Objects

    vlm_response = vlm_client.text_query(text_prompt, UserSemanticTarget.model_json_schema())
    user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)

    img = Image.open(r"test_data\vlm_test_imgs\img4.png")
    img = np.asarray(img)
    response = vlm_client.image_text_query(img, ASSIGN_ROOM_LABEL_ONLY_PROMPT)
    print(response)

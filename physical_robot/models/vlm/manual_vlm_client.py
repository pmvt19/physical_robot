import numpy as np
import matplotlib.pyplot as plt

from physical_robot.models.vlm.vlm_client import AbstractVLMClient

class ManualSimulatedVLMClient(AbstractVLMClient):
    def __init__(self):
        pass

    def _image_text_query_schema(self, image_prompt: np.ndarray, text_prompt: str, schema: dict):
        raise NotImplementedError
        
    def _image_text_query_no_schema(self, image_prompt: np.ndarray, text_prompt: str):
        plt.imshow(image_prompt)
        plt.title("Image Prompt for VLM Output")
        plt.show()

        user_input = input(f"What is the response you would like the VLM to give? To the following prompt and the previous image: {text_prompt}")
        return user_input

    def _text_query_schmea(self, text_prompt: str, schema: dict):
        raise NotImplementedError
        
    def _text_query_no_schmea(self, text_prompt: str):
        user_input = input(f"What is the response you would like the VLM to give? To the following prompt: {text_prompt}")
        return user_input
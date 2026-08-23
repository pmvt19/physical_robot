import numpy as np
import matplotlib.pyplot as plt

from physical_robot.models.vlm.vlm_client import AbstractVLMClient

class ManualSimulatedVLMClient(AbstractVLMClient):
    def __init__(self):
        pass

    def _format_to_schema(self, user_input: str, schema: dict):
        # TODO: Process Schema
        pass

    def _image_text_query_schema(self, image_prompt: np.ndarray, text_prompt: str, schema: dict):
        plt.imshow(image_prompt)
        plt.title("Image Prompt for VLM Output (with schema)")
        plt.show()

        user_input = input(f"What is the response you would like the VLM to give? To the following prompt and the previous image: {text_prompt}. This output must follow the following schema: {schema}")
        
        formatted_output = self._format_to_schema(user_input, schema)
        return formatted_output

    def _image_text_query_no_schema(self, image_prompt: np.ndarray, text_prompt: str):
        plt.imshow(image_prompt)
        plt.title("Image Prompt for VLM Output")
        plt.show()

        user_input = input(f"What is the response you would like the VLM to give? To the following prompt and the previous image: {text_prompt}")
        return user_input

    def _text_query_schmea(self, text_prompt: str, schema: dict):
        user_input = input(f"What is the response you would like the VLM to give? "
                           f"To the following prompt: {text_prompt}. "
                           f"This output must follow the following schema: {schema}")

        formatted_output = self._format_to_schema(user_input, schema)
        return formatted_output
        
    def _text_query_no_schmea(self, text_prompt: str):
        user_input = input(f"What is the response you would like the VLM to give? To the following prompt: {text_prompt}")
        return user_input
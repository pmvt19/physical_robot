import numpy as np
from PIL import Image
import io

# TODO: Add Base Class??
class AbstractVLMClient():
    def __init__(self):
        pass

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
        raise NotImplementedError
    
    def _image_text_query_no_schema(self, image_prompt: np.ndarray, text_prompt: str):
        raise NotImplementedError
    
    def image_text_query(self, image_prompt: np.ndarray, text_prompt: str, schema: dict = None):
        if schema is not None:
            # Query Using Schema
            return self._image_text_query_schema(image_prompt=image_prompt, text_prompt=text_prompt, schema=schema)
        else:
            # Query Using No Schema
            return self._image_text_query_no_schema(image_prompt=image_prompt, text_prompt=text_prompt)

    def _text_query_schmea(self, text_prompt: str, schema: dict):
        raise NotImplementedError
    
    def _text_query_no_schmea(self, text_prompt: str):
        raise NotImplementedError

    def text_query(self, text_prompt: str, schema: dict = None):
        if schema is not None:
            # Query Using Schema
            return self._text_query_schmea(text_prompt=text_prompt, schema=schema)
        else:
            # Query Using No Schema
            return self._text_query_no_schmea(text_prompt=text_prompt)
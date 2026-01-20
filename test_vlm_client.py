import numpy as np

from PIL import Image

from vlm_client import VLMClient
from prompts import EXTRACT_SEMANTIC_TARGETS, ASSIGN_ROOM_LABEL
from vlm_output_schema import UserSemanticTarget, RoomLabel

def test_text_prompt_no_schema():
    vlm_client = VLMClient()

    vlm_response = vlm_client.text_query("What is the room where you would want to cook your meals?")
    assert 'kitchen' in vlm_response.text

def test_text_prompt_schema():
    vlm_client = VLMClient()

    room_list = ['bedroom', 'kitchen', 'office']
    object_list = ['bed', 'oven', 'desk', 'chair']
    invalid_object_list = ['person', 'sky']

    vlm_response = vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format("Let's go towards the kitchen", 
                                                                             room_list,
                                                                             object_list,
                                                                             invalid_object_list),
                                        UserSemanticTarget.model_json_schema())

    user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response.text)
    assert user_semantic_target.semantic_level == 'room'
    assert user_semantic_target.valid == True
    assert user_semantic_target.item_name == "kitchen"

    vlm_response = vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format("Let's go towards the desk", 
                                                                             room_list,
                                                                             object_list,
                                                                             invalid_object_list),
                                        UserSemanticTarget.model_json_schema())

    user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response.text)
    assert user_semantic_target.semantic_level == 'object'
    assert user_semantic_target.valid == True
    assert user_semantic_target.item_name == "desk"

    vlm_response = vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format("DKJKLSDFKJL", 
                                                                             room_list,
                                                                             object_list,
                                                                             invalid_object_list),
                                        UserSemanticTarget.model_json_schema())

    user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response.text)
    
    assert user_semantic_target.valid == False
    assert user_semantic_target.semantic_level == 'N/A'
    assert user_semantic_target.item_name == "N/A"

def test_image_text_prompt_no_schema():
    vlm_client = VLMClient()

    img = Image.open('/Users/pravaltelagi/oakd_camera/saves/apartment/rgb_imgs/0004.png')
    img = np.asarray(img)
    vlm_response = vlm_client.image_text_query(img, "What is this an image of?")

    assert 'kitchen' in vlm_response.text

def test_image_text_prompt_schema():
    vlm_client = VLMClient()

    img = Image.open('/Users/pravaltelagi/oakd_camera/saves/apartment/rgb_imgs/0004.png')
    img = np.asarray(img)

    existing_rooms = ['kitchen', 'office' 'bedroom']
    invalid_rooms = ['wall', 'room']

    vlm_response = vlm_client.image_text_query(img, 
                                               ASSIGN_ROOM_LABEL.format(
                                                    existing_rooms,
                                                    invalid_rooms
                                                ), 
                                                RoomLabel.model_json_schema())

    room_label = RoomLabel.model_validate_json(vlm_response.text)

    assert room_label.room_label == 'kitchen'

if __name__ == '__main__':
    test_text_prompt_no_schema()
    test_text_prompt_schema()
    test_image_text_prompt_no_schema()
    test_image_text_prompt_schema()
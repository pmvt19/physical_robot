import unittest

import numpy as np
from PIL import Image

from physical_robot.models.vlm.gemma_vlm_client import OllamaVLMClient
from physical_robot.models.vlm.prompts import (
    ASSIGN_ROOM_LABEL,
    EXTRACT_SEMANTIC_TARGETS,
)
from physical_robot.models.vlm.vlm_output_schema import RoomLabel, UserSemanticTarget


class TestVLMClients(unittest.TestCase):
    def setUp(self):
        self.vlm_client = OllamaVLMClient()

        self.room_list = ['bedroom', 'kitchen', 'office']
        self.object_list = ['bed', 'oven', 'desk', 'chair']
        self.invalid_object_list = ['person', 'sky']
        
    def test_text_prompt_no_schema(self):
        vlm_response = self.vlm_client.text_query(
            "What is the room where you would cook your meals?"
        )
        self.assertIn("kitchen", vlm_response)

    def test_text_prompt_schema_valid_room(self):
        vlm_response = self.vlm_client.text_query(
            EXTRACT_SEMANTIC_TARGETS.format(
                "Let's go towards the kitchen", 
                self.room_list, self.object_list, self.invalid_object_list
            ),
            UserSemanticTarget.model_json_schema()
        )

        user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)

        self.assertEqual(user_semantic_target.semantic_level, "room")
        self.assertTrue(user_semantic_target.valid)
        self.assertEqual(user_semantic_target.item_name, "kitchen")
    
    def test_text_prompt_schema_valid_object(self):
        vlm_response = self.vlm_client.text_query(
            EXTRACT_SEMANTIC_TARGETS.format(
                "Let's go towards the desk", self.room_list,
                self.object_list, self.invalid_object_list
            ),
            UserSemanticTarget.model_json_schema()
        )

        user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)

        self.assertEqual(user_semantic_target.semantic_level, "object")
        self.assertTrue(user_semantic_target.valid)
        self.assertEqual(user_semantic_target.item_name, "desk")

    def test_text_prompt_schema_invalid_object(self):
        vlm_response = self.vlm_client.text_query(
            EXTRACT_SEMANTIC_TARGETS.format(
                "DKJKLSDFKJL",
                self.room_list,
                self.object_list,
                self.invalid_object_list
            ),
            UserSemanticTarget.model_json_schema()
        )

        user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)
    
        self.assertFalse(user_semantic_target.valid)
        self.assertEqual(user_semantic_target.semantic_level, "N/A")
        self.assertEqual(user_semantic_target.item_name, "N/A")

    def test_image_text_prompt_no_schema(self):
        img = Image.open('test_data/vlm_test_imgs/img0.png')
        img = np.asarray(img)
        vlm_response = self.vlm_client.image_text_query(
            img, "What is this an image of?"
        )

        self.assertIn("kitchen", vlm_response)

    def test_image_text_prompt_schema(self):
        img = Image.open('test_data/vlm_test_imgs/img0.png')
        img = np.asarray(img)

        existing_rooms = ['kitchen', 'office' 'bedroom']
        invalid_rooms = ['wall', 'room']

        vlm_response = self.vlm_client.image_text_query(
            img, 
            ASSIGN_ROOM_LABEL.format(
                existing_rooms, invalid_rooms
            ), 
            RoomLabel.model_json_schema()
        )

        room_label = RoomLabel.model_validate_json(vlm_response)

        self.assertEqual(room_label.room_label, "kitchen")
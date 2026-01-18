from robot import Robot
from basic_map import BasicMap
from map import Map
from advanced_map import AdvancedMap
from semantic_map import SemanticMap
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
import os
from vlm_client import VLMClient
from image_segmentation import ImageSegmenter
from prompts import ASSIGN_ROOM_LABEL_ONLY_PROMPT, ASSIGN_ROOM_LABEL
from vlm_output_schema import RoomLabel

# from config import scene_name

"""
saves/
    -scenes/
        -scene_name/
            -map.pickle
            -prm.pickle
            -incremental_imgs/
"""

def visualize_all_maps(map: Map, advanced_map: AdvancedMap, semantic_map: SemanticMap):
    fig, ax = plt.subplots(1, 3)
    map.visualize(ax=ax[0])
    advanced_map.visualize(ax=ax[1])
    semantic_map.visualize(ax=ax[2])
    plt.show()

def visualize_semantics(semantic_map: SemanticMap):
    fig, ax = plt.subplots(1, 2)
    semantic_map.visualize_semantic_layer(ax[0], layer='room')
    semantic_map.visualize_semantic_layer(ax[1], layer='object')
    print(semantic_map.room_to_id)
    print(semantic_map.object_to_id)
    plt.show()

if __name__ == "__main__":

    ## TODO: Will update directory structure soon
    scene_name = 'extensive_apartment'
    map_save_dir = f'saves/scenes/{scene_name}'

    # Initialization
    robot = Robot(connection='client')
    
    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)

    # # Initialize Maps
    # Basic Map
    map = Map()
    map.init_map(initial_scan=scan)

    # Advanced Map
    advanced_map = AdvancedMap()
    advanced_map.init_map(initial_scan=scan)

    # Advanced Map for Semantics
    semantic_map_advanced_map = AdvancedMap()
    # Semantic Map
    semantic_map = SemanticMap(map_obj=semantic_map_advanced_map) # Needs to be Initialized with another map
    semantic_map.init_map(initial_scan=scan) # Should be here

    i=0

    # Initialize ML Clients
    image_segmenter = ImageSegmenter()
    vlm_client = VLMClient()

    # Initialize Starting States for Each Map
    map_state = np.array([0.0, 0.0, 0.0])
    advanced_map_state = np.array([0.0, 0.0, 0.0])
    semantic_map_state = np.array([0.0, 0.0, 0.0])

    while True:
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break
        
        m = robot.command_motion_trial(motion_command)
        
        # Predict State for each Map
        map_predicted_state = robot.predict_state(map_state, m)
        advanced_map_predicted_state = robot.predict_state(advanced_map_state, m)
        semantic_map_predicted_state = robot.predict_state(semantic_map_state, m)

        # Read Lidar
        scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)

        # Read Camera
        rgb_img, _ = robot.read_rgb_camera()

        # Read Point Cloud
        pc, _ = robot.read_point_cloud()

        # Update map and get updated state
        map_updated_state = map.update(scan, map_predicted_state)
        advanced_map_updated_state = advanced_map.update(scan, advanced_map_predicted_state)

        # # Semantic Update

        # Get Room Label
        room_label_response = vlm_client.image_text_query(rgb_img,
                                                          ASSIGN_ROOM_LABEL.format(
                                                              semantic_map.get_room_list(), 
                                                              semantic_map.get_invalid_room_list()),
                                                          RoomLabel.model_json_schema())
        room_label = RoomLabel.model_validate_json(room_label_response.text)

        # Get Image Segmentation
        prediction, labels = image_segmenter.segment_image(rgb_img)

        # Format Image Segmentation
        formatted_segmented_img = semantic_map.format_img_segmentation(prediction['segmentation'], labels.items())

        # Format PC to What Semantic Map Wants
        pc_and_labels = semantic_map.label_and_filter_point_cloud(pc, formatted_segmented_img, room_label.room_label)

        # Update Semantic Map with Geometry and Semantics
        semantic_map_updated_state = semantic_map.update_geometry_and_semantics(scan, pc_and_labels, semantic_map_predicted_state)

        # Update the states for each map
        map_state = map_updated_state
        advanced_map_state = advanced_map_updated_state
        semantic_map_state = semantic_map_updated_state

        i+=1

        # Save All Geometric Maps and Semantic Layers in SemanticMap
        visualize_all_maps(map, advanced_map, semantic_map)
        visualize_semantics(semantic_map)

        # Save All Maps
        map.save(map_save_dir)
        advanced_map.save(map_save_dir)
        semantic_map.save(map_save_dir)


    

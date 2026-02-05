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

from map_builder import MapBuilder, AdvancedMapBuilder, SemanticMapBuilder
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
    scene_name = 'slam_maps_map_builder'
    map_save_dir = f'saves/scenes/{scene_name}'

    # Initialization
    robot = Robot(connection='client')
    
    # # Initialize Map Builder
    # map_builder = SemanticMapBuilder(robot=robot)
    # map_builder.init()

    # while True:
    #     # Get the Motion Command from the User
    #     motion_command = robot.request_motion_command_from_user()
    #     if motion_command[0] == '': # No Motion Command
    #         break
        
    #     # Move the Robot
    #     m = robot.command_motion_trial(motion_command)

    #     # Step the Map Building        
    #     map_builder.step(m)

    #     # Display Map
    #     map_builder.show()

    #     # TODO: Convert to get map function
    #     map_builder.map.save(map_save_dir=map_save_dir)

    build_map = True
    build_advanced_map = True
    build_semantic_map = True

    map_builders: list[MapBuilder] = []

    if build_map:
        map_builders.append(MapBuilder())

    if build_advanced_map:
        map_builders.append(AdvancedMapBuilder())

    if build_semantic_map:
        map_builders.append(SemanticMapBuilder())

    # Init Maps
    [map_builder.init() for map_builder in map_builders]

    while True:
        # Get the Motion Command from the User
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break
        
        # Move the Robot
        m = robot.command_motion_trial(motion_command)

        for map_builder in map_builders:

            # Step the Map Building        
            map_builder.step(m)

            # Display Map
            map_builder.show()

            # Save Map
            map_builder.get_map().save(map_save_dir=map_save_dir)

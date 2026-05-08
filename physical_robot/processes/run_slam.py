from physical_robot.robot import Robot
from physical_robot.maps import Map, AdvancedMap, SemanticMap
import numpy as np
import matplotlib.pyplot as plt

from physical_robot.map_builder.map_builder import MapBuilder, AdvancedMapBuilder, SemanticMapBuilder
# from config import scene_name

"""
saves/
    -scenes/
        -scene_name/
            -map.pickle
            -prm.pickle
            -incremental_imgs/
"""

# TODO: Make robust visualization function here
# You just pass it map_builders list and it automatically shows you all the geometry maps
# and semantic maps if applicable
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

def visualize_map_builders(map_builders: list[MapBuilder], viz_semantics=True):
    num_map_builders = len(map_builders)

    # Should never be practically more than 3 (Only 3 types of maps as of now)
    fig, ax = plt.subplots(1, num_map_builders)

    for i, map_builder in enumerate(map_builders):
        map_builder.get_map().visualize(ax=ax[i])
    plt.show()

    if viz_semantics:
        # Individualy display the semantics if enabled
        for i, map_builder in enumerate(map_builders):
            if isinstance(map_builder, SemanticMapBuilder):
                semantic_map: SemanticMap = map_builder.get_map()

                fig, ax = plt.subplots(1, 2)
                semantic_map.visualize_semantic_layer(ax[0], layer='room')
                semantic_map.visualize_semantic_layer(ax[1], layer='object')
                print(semantic_map.room_to_id)
                print(semantic_map.object_to_id)
                plt.show()

if __name__ == "__main__":

    ## TODO: Make Flag?
    scene_name = 'refactor_test_map'
    map_save_dir = f'saves/scenes/{scene_name}'

    # Initialization
    robot = Robot(connection='client')
    
    # TODO: Convert to flags??
    build_map = False
    build_advanced_map = False
    build_semantic_map = True

    map_builders: list[MapBuilder] = []

    if build_map:
        map_builders.append(MapBuilder(robot))

    if build_advanced_map:
        map_builders.append(AdvancedMapBuilder(robot))

    if build_semantic_map:
        map_builders.append(SemanticMapBuilder(robot))

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

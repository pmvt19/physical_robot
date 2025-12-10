from robot import Robot
from basic_map import BasicMap
from map import Map
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
import os
from image_segmentation import ImageSegmenter
import rerun as rr
from semantic_map import SemanticMap

"""
This file is not used for running slam yet, but actually used for generating semantic information for point clouds
"""

def get_image_semantics(rgb_img):
    """
    Given an RGB image, return semantic information for the image

    This is a placeholder function. Actual implementation will depend on the semantic segmentation model used.
    """
    # For now, return dummy semantics
    semantics = None
    return semantics

def process_labels():
    """
    Figure out what labels are useful
    """

def identify_room():
    """
    Identify what room we are looking at based on image
    """

def init_directories(top_level_dir):
    os.makedirs(f'{top_level_dir}/semantic_map_imgs', exist_ok=True)
    os.makedirs(f'{top_level_dir}/semantic_map', exist_ok=True)

    os.makedirs(f'{top_level_dir}/geometric_map_imgs', exist_ok=True)
    os.makedirs(f'{top_level_dir}/geometric_map', exist_ok=True)

def label_filtered_pc(image_segmenter : ImageSegmenter, semantic_map : SemanticMap, pc, prediction, prompts):
    all_instance_labeled_filtered_pc = np.empty((0, 4))
    for prompt in prompts:
        object_id = semantic_map.get_object_id(prompt)
        mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt=prompt).flatten() # id: object_id
        instance_labeled_filtered_pc = np.hstack((pc[mask], np.ones((len(pc[mask]), 1))*object_id))
        all_instance_labeled_filtered_pc = np.concatenate((all_instance_labeled_filtered_pc, instance_labeled_filtered_pc), axis=0)

    return all_instance_labeled_filtered_pc

def semantic_slam():

    scene_name = 'semantic_apartment'
    map_save_dir = f'saves/scenes/{scene_name}'
    init_directories(map_save_dir)

    image_segmenter = ImageSegmenter()

    robot = Robot(connection='client')
    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)
    map = Map(initial_scan=scan)
    semantic_map = SemanticMap(map)

    object_list = ['oven', 'cabinet', 'table', 'backpack', 'bed', 'refrigerator', 'tv', 'window', 'bottle', 'chair', 'clothes']

    i = 0
    while True:
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break

        m = robot.command_motion_trial(motion_command)
        predicted_state = robot.predict_state(robot.state, m)
        robot.state = predicted_state
        print("Predicted State", robot.state)

        # Read Robot Sensors: Lidar, RGBD Camera/Point Cloud
        lidar_coords, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)
        rgb_img, _ = robot.read_rgb_camera()
        pc, colors = robot.read_point_cloud()

        print("Segmenting Images")
        prediction, labels = image_segmenter.segment_image(rgb_img)

        # TESTING ONLY VISUALIZATION
        # image_segmenter.draw_panoptic_segmentation(plt.gca(), prediction['segmentation'], prediction['segments_info'])
        # TESTING ONLY VISUALIZATION

        filtered_pc = label_filtered_pc(image_segmenter, semantic_map, pc, prediction, object_list)
        pc_flattened_coords = np.stack((filtered_pc[:, 0], filtered_pc[:, 2]), axis=1)
        print("Finished Filtering PC")

        # TODO: HACK Address this hack : #rotate 90 degrees clockwise
        theta = -np.pi / 2  # 90 degrees in radians
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                     [np.sin(theta), np.cos(theta)]])
        pc_flattened_coords = pc_flattened_coords.dot(rotation_matrix.T)
        # TODO: HACK rotation done

        pc_flattened_coords_and_labels = np.concatenate((pc_flattened_coords, filtered_pc[:, 3:4].astype(np.int64)), axis=1)

        updated_state = semantic_map.update(lidar_coords, pc_flattened_coords_and_labels, predicted_state)
        robot.state = updated_state
        print("Updated State", robot.state)

        print(semantic_map.object_to_id)

        fig, ax = plt.subplots(1, 2)
        semantic_map.visualize(ax, layer='room')
        plt.savefig(f'{map_save_dir}/semantic_map_imgs/semantic_map_{i}.png')

        # fig, ax = plt.subplots(1, 2)
        # semantic_map.visualize(ax, layer='room')
        # plt.show()

        pickle.dump(semantic_map.map, open(f"{map_save_dir}/geometric_map/map_object.pickle", "wb"))
        pickle.dump(semantic_map.map.map, open(f"{map_save_dir}/geometric_map/map_map.pickle", "wb"))
        pickle.dump(semantic_map.map.get_points(), open(f"{map_save_dir}/geometric_map/map_points.pickle", "wb"))

        pickle.dump(semantic_map, open(f"{map_save_dir}/semantic_map/semantic_map_object.pickle", "wb"))
        pickle.dump(semantic_map.semantic_layer, open(f"{map_save_dir}/semantic_map/semantic_layer.pickle", "wb"))
        pickle.dump(semantic_map.object_to_id, open(f"{map_save_dir}/semantic_map/semantic_info.pickle", "wb"))
        print("Done Saving Maps")
        # fig, ax = plt.subplots(1, 3)
        # semantic_map.flood_fill(limit_fill_extent=True, method='nearest_neighbor')
        # semantic_map.visualize(ax, layer='room')
        # plt.show()
        i += 1




if __name__ == "__main__":
    semantic_slam()
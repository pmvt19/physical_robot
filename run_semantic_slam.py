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

def label_filtered_pc(image_segmenter : ImageSegmenter, semantic_map : SemanticMap, pc, prediction, prompts):
    all_instance_labeled_filtered_pc = np.empty((0, 4))
    for prompt in prompts:
        object_id = semantic_map.get_object_id(prompt)
        mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt=prompt).flatten() # id: object_id
        instance_labeled_filtered_pc = np.hstack((pc[mask], np.ones((len(pc[mask]), 1))*object_id))
        all_instance_labeled_filtered_pc = np.concatenate((all_instance_labeled_filtered_pc, instance_labeled_filtered_pc), axis=0)

    return all_instance_labeled_filtered_pc

def semantic_slam():
    image_segmenter = ImageSegmenter()

    robot = Robot(connection='client')
    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)
    map = Map(initial_scan=scan)
    semantic_map = SemanticMap(map)

    object_list = ['oven', 'cabinet', 'table', 'backpack', 'bed', 'refrigerator', 'tv', 'window']

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

        prediction, labels = image_segmenter.segment_image(rgb_img)

        # TESTING ONLY VISUALIZATION
        image_segmenter.draw_panoptic_segmentation(plt.gca(), prediction['segmentation'], prediction['segments_info'])
        # TESTING ONLY VISUALIZATION

        filtered_pc = label_filtered_pc(image_segmenter, semantic_map, pc, prediction, object_list)
        pc_flattened_coords = np.stack((filtered_pc[:, 0], filtered_pc[:, 2]), axis=1)

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
        plt.show()

        # fig, ax = plt.subplots(1, 3)
        # semantic_map.flood_fill(limit_fill_extent=True, method='nearest_neighbor')
        # semantic_map.visualize(ax, layer='room')
        # plt.show()




if __name__ == "__main__":
    semantic_slam()
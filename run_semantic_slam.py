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


def semantic_slam():
    image_segmenter = ImageSegmenter()

    robot = Robot(connection='client')
    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)
    map = Map(initial_scan=scan)
    semantic_map = SemanticMap(map)

    while True:
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break

        m = robot.command_motion_trial(motion_command)
        predicted_state = robot.predict_state(robot.state, m)

        # Read Robot Sensors: Lidar, RGBD Camera/Point Cloud
        lidar_coords, _ = robot.read_lidar_updated(manual_verification=False, wait_for_updated_reading=True)
        rgb_img, _ = robot.read_rgb_camera()
        pc, colors = robot.read_point_cloud()

        prediction, labels = image_segmenter.segment_image(rgb_img)

        # TESTING ONLY VISUALIZATION
        image_segmenter.draw_panoptic_segmentation(plt.gca(), prediction['segmentation'], prediction['segments_info'])
        plt.title("Panoptic Segmentation: TESTING ONLY")
        plt.show()
        # TESTING ONLY VISUALIZATION

        oven_mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt='oven').flatten() # id: 0
        cabinet_mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt='cabinet').flatten() # id: 1
        desk_mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt='table').flatten() # id: 2

        filtered_pc_oven = np.concatenate(pc[oven_mask], np.zeros((len(pc[oven_mask]), 1)).astype(np.int32))
        filtered_pc_cabinet = np.concatenate(pc[oven_mask], np.ones((len(pc[cabinet_mask]), 1)).astype(np.int32))
        filtered_pc_desk = np.concatenate(pc[oven_mask], (np.ones((len(pc[desk_mask]), 1))*2).astype(np.int32))

        filtered_pc = np.concatenate((filtered_pc_oven, filtered_pc_cabinet, filtered_pc_desk), axis=0)
        pc_flattened_coords = np.stack((filtered_pc[:, 0], filtered_pc[:, 2]), axis=1)
        # pc_flattened_coords_and_labels = np.stack((filtered_pc[:, 0], filtered_pc[:, 2], filtered_pc[:, 3]), axis=1)

        # TODO: Address this hack : #rotate 90 degrees clockwise
        theta = -np.pi / 2  # 90 degrees in radians
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                     [np.sin(theta), np.cos(theta)]])
        pc_flattened_coords = pc_flattened_coords.dot(rotation_matrix.T)
        pc_flattened_coords_and_labels = np.concatenate((pc_flattened_coords, filtered_pc[:, 3:4]), axis=1)

        updated_state = semantic_map.update(lidar_coords, pc_flattened_coords_and_labels, predicted_state)
        robot.state = updated_state

        semantic_map.visualize()




if __name__ == "__main__":
    start_time = time.time()

    image_segmenter = ImageSegmenter()

    robot = Robot(connection='client')

    scan, _ = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)
    map = Map(initial_scan=scan)

    while True:
        motion_command = robot.request_motion_command_from_user()
        if motion_command[0] == '': # No Motion Command
            break

        m = robot.command_motion_trial(motion_command)
        predicted_state = robot.predict_state(robot.state, m)

        scan, _ = robot.read_lidar_updated(manual_verification=False, wait_for_updated_reading=True)
        updated_state = map.update(scan, predicted_state)
        robot.state = updated_state

        rgb_img, _ = robot.read_rgb_camera()
        pc, colors = robot.read_point_cloud()

        prediction, labels = image_segmenter.segment_image(rgb_img)
        print(labels)
        image_segmenter.draw_panoptic_segmentation(plt.gca(), prediction['segmentation'], prediction['segments_info'])
        plt.show()

        oven_mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt='oven').flatten()
        cabinet_mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt='cabinet').flatten()
        desk_mask = image_segmenter.get_instance_segment_mask(prediction['segmentation'], prediction['segments_info'], prompt='table').flatten()

        print(oven_mask.dtype, cabinet_mask.dtype, desk_mask.dtype)
        print(oven_mask.sum(), cabinet_mask.sum(), desk_mask.sum())

        keep_point_mask = np.logical_or.reduce([oven_mask, cabinet_mask, desk_mask])
        print("keep points:", keep_point_mask.sum(), "out of", keep_point_mask.shape[0])
        print(keep_point_mask.dtype, keep_point_mask.shape, pc[keep_point_mask].shape, pc.shape)
        pc = pc[keep_point_mask]
        colors = colors[keep_point_mask]
        print("Filtered PC Shape:", pc.shape)

        map.visualize(plt.gca())
        plt.show()

        # test_map = np.zeros_like(map.map)
        test_map = np.copy(map.map)
        N, M = test_map.shape
        pc_flattened = np.stack((pc[:, 0], pc[:, 2]), axis=1)
        #rotate 90 degrees clockwise
        theta = -np.pi / 2  # 90 degrees in radians
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                     [np.sin(theta), np.cos(theta)]])
        pc_flattened = pc_flattened.dot(rotation_matrix.T)
        pc_grid_coords = map.batch_world_to_grid_coords(pc_flattened)

        # valid_mask = (pc_grid_coords[:, 0] >= 0) & (pc_grid_coords[:, 0] < N) & (pc_grid_coords[:, 1] >= 0) & (pc_grid_coords[:, 1] < M)
        valid_mask = np.logical_and.reduce((
            pc_grid_coords[:, 0] >= 0,
            pc_grid_coords[:, 0] < N,
            pc_grid_coords[:, 1] >= 0,
            pc_grid_coords[:, 1] < M
        ))
        print("valid points:", valid_mask.sum(), "out of", pc_grid_coords.shape[0])
        pc_grid_coords = pc_grid_coords[valid_mask]

        print(pc_grid_coords.shape)

        test_map[pc_grid_coords[:, 0], pc_grid_coords[:, 1]] = 0.5

        plt.imshow(np.rot90(test_map))
        plt.title("Mapped Semantic Points")
        plt.show()
        exit()

        




    

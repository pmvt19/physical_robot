import numpy as np
import matplotlib.pyplot as plt

from robot import Robot

from map import Map
from advanced_map import AdvancedMap
from semantic_map import SemanticMap

from vlm_client import VLMClient
from image_segmentation import ImageSegmenter

from prompts import ASSIGN_ROOM_LABEL
from vlm_output_schema import RoomLabel


class MapBuilder():
    def __init__(self, robot, manual_lidar_verification=False):
        self.robot: Robot = robot
        self.manual_lidar_verification = manual_lidar_verification
        self.robot_state = np.array([0.0, 0.0, 0.0])

        self.map: Map = Map()

    def init(self):
        world_coords, raw_lidar_data = self.robot.read_lidar_updated(manual_verification=self.manual_lidar_verification, wait_for_updated_reading=True)
        self.map.init_map(initial_scan=world_coords)
    
    def step(self, m):
        self.robot_state = self.robot.predict_state(self.robot_state, m)

        # Read Lidar
        world_coords, raw_lidar_data = self.robot.read_lidar_updated(manual_verification=self.manual_lidar_verification, wait_for_updated_reading=True)

        # Update Map with New Geometry
        self.robot_state = self.map.update(world_coords, self.robot_state)

    def get_map(self):
        return self.map
    
    def get_robot_state(self):
        return self.robot_state
    
    def show(self):
        self.map.visualize(plt.gca())
        plt.show()

class AdvancedMapBuilder(MapBuilder):
    def __init__(self, robot, manual_lidar_verification=False):
        super().__init__(robot, manual_lidar_verification)

        self.map: AdvancedMap = AdvancedMap()

class SemanticMapBuilder(MapBuilder):
    def __init__(self, robot, manual_lidar_verification=False):
        super().__init__(robot, manual_lidar_verification)

        self.map: SemanticMap = SemanticMap(map_obj=AdvancedMap())

        # Initialize ML Clients
        self.image_segmenter = ImageSegmenter()
        self.vlm_client = VLMClient()

    def step(self, m):
        self.robot_state = self.robot.predict_state(self.robot_state, m)

        # Read Lidar
        world_coords, raw_lidar_data = self.robot.read_lidar_updated(manual_verification=self.manual_lidar_verification, wait_for_updated_reading=True)

        # Read Camera
        rgb_img, _ = self.robot.read_rgb_camera()

        # Read Point Cloud
        pc, _ = self.robot.read_point_cloud()

        # Query the VLM for the Room Label
        room_label_response = self.vlm_client.image_text_query(rgb_img,
                                                          ASSIGN_ROOM_LABEL.format(
                                                              self.map.get_room_list(), 
                                                              self.map.get_invalid_room_list()),
                                                          RoomLabel.model_json_schema())
        # TODO: To make this interoperable with other VLMClient outputs, return the .text already?
        room_label = RoomLabel.model_validate_json(room_label_response.text)

        # Get Image Segmentation
        prediction, labels = self.image_segmenter.segment_image(rgb_img)

        # Format Image Segmentation
        formatted_segmented_img = self.map.format_img_segmentation(prediction['segmentation'], labels.items())

        # Format PC to What Semantic Map Wants
        pc_and_labels = self.map.label_and_filter_point_cloud(pc, formatted_segmented_img, room_label.room_label)

        # Update Semantic Map with Geometry and Semantics
        self.robot_state = self.map.update_geometry_and_semantics(world_coords, pc_and_labels, self.robot_state)

    def show(self):
        fig, ax = plt.subplots(1, 3)
        self.map.visualize(ax[0])
        self.map.visualize_semantic_layer(ax[1], layer='room')
        self.map.visualize_semantic_layer(ax[2], layer='object')
        print(self.map.room_to_id)
        print(self.map.object_to_id)
        plt.show()

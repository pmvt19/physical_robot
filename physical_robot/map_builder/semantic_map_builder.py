import matplotlib.pyplot as plt

from physical_robot.map_builder import MapBuilder
from physical_robot.maps import AdvancedMap, SemanticMap
from physical_robot.models.segmentation.image_segmentation import ImageSegmenter
from physical_robot.models.vlm.ollama_vlm_client import OllamaVLMClient
from physical_robot.models.vlm.prompts import ASSIGN_ROOM_LABEL
from physical_robot.models.vlm.vlm_output_schema import RoomLabel


class SemanticMapBuilder(MapBuilder):
    def __init__(self, robot, map_resolution=10.0, manual_lidar_verification=False):
        super().__init__(robot, map_resolution, manual_lidar_verification)

        # Initialize Semantic Map
        self.map: SemanticMap = SemanticMap(
            map_obj=AdvancedMap(resolution=map_resolution)
        )

        # Initialize ML Clients
        self.image_segmenter = ImageSegmenter()
        self.vlm_client = OllamaVLMClient()

    def step(self, m):
        self.robot_state = self.robot.predict_state(self.robot_state, m)

        # Read Lidar
        world_coords, _ = self.robot.read_lidar_updated(
            manual_verification=self.manual_lidar_verification,
            wait_for_updated_reading=True,
        )

        # Read Camera
        rgb_img, _ = self.robot.read_rgb_camera()

        # Read Point Cloud
        pc, _ = self.robot.read_point_cloud()

        # Query the VLM for the Room Label
        room_label_response = self.vlm_client.image_text_query(
            rgb_img,
            ASSIGN_ROOM_LABEL.format(
                self.map.get_room_list(), self.map.get_invalid_room_list()
            ),
            RoomLabel.model_json_schema(),
        )
        room_label = RoomLabel.model_validate_json(room_label_response)

        # Get Image Segmentation
        prediction, labels = self.image_segmenter.segment_image(rgb_img)

        # Format Image Segmentation
        formatted_segmented_img = self.map.format_img_segmentation(
            prediction["segmentation"], labels.items()
        )

        # Format PC to What Semantic Map Wants
        pc_and_labels = self.map.label_and_filter_point_cloud(
            pc, formatted_segmented_img, room_label.room_label
        )

        # Update Semantic Map with Geometry and Semantics
        self.robot_state = self.map.update_geometry_and_semantics(
            world_coords, pc_and_labels, self.robot_state
        )

        # Add Robot State to Trajectory
        self.robot_trajectory.append(self.robot_state)

    def show(self):
        fig, ax = plt.subplots(1, 3)
        self.map.visualize(ax[0])
        self.map.visualize_semantic_layer(ax[1], layer="room")
        self.map.visualize_semantic_layer(ax[2], layer="object")
        print(self.map.room_to_id)
        print(self.map.object_to_id)
        plt.show()

import numpy as np
import matplotlib.pyplot as plt

from robot import Robot

from map import Map
from advanced_map import AdvancedMap
from semantic_map import SemanticMap




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

        world_coords, raw_lidar_data = self.robot.read_lidar_updated(manual_verification=self.manual_lidar_verification, wait_for_updated_reading=True)
        self.robot_state = self.map.update(world_coords, self.robot_state)

    def show(self):
        self.map.visualize(plt.gca())

class AdvancedMapBuilder(MapBuilder):
    def __init__(self, manual_lidar_verification=False):
        super().__init__(manual_lidar_verification)

        self.map: AdvancedMap = AdvancedMap()

    def step(self, m):
        pass

class SemanticMapBuilder(MapBuilder):
    def __init__(self, manual_lidar_verification=False):
        super().__init__(manual_lidar_verification)

        self.map: SemanticMap = SemanticMap()

    def step(self, m):
        pass

    def show(self):
        fig, ax = plt.subplots(1, 3)
        self.map.visualize(ax[0])
        self.map.visualize_semantic_layer(ax[1], layer='room')
        self.map.visualize_semantic_layer(ax[2], layer='object')
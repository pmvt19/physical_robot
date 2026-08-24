import matplotlib.pyplot as plt
import numpy as np

from physical_robot.maps import Map
from physical_robot.robot import Robot


class MapBuilder:
    def __init__(self, robot, map_resolution=10.0, manual_lidar_verification=False):
        self.robot: Robot = robot
        self.manual_lidar_verification = manual_lidar_verification
        self.robot_state = np.array([0.0, 0.0, 0.0])

        self.map: Map = Map(resolution=map_resolution)

        self.robot_trajectory: list[np.ndarray] = []

    def init(self):
        world_coords, _ = self.robot.read_lidar_updated(
            manual_verification=self.manual_lidar_verification,
            wait_for_updated_reading=True,
        )
        self.map.init_map(initial_scan=world_coords)

    def step(self, m):
        self.robot_state = self.robot.predict_state(self.robot_state, m)

        # Read Lidar
        world_coords, _ = self.robot.read_lidar_updated(
            manual_verification=self.manual_lidar_verification,
            wait_for_updated_reading=True,
        )

        # Update Map with New Geometry
        self.robot_state = self.map.update(world_coords, self.robot_state)
        self.robot_trajectory.append(self.robot_state)

    def get_map(self):
        return self.map

    def get_robot_state(self):
        return self.robot_state

    def get_robot_trajectory(self):
        return self.robot_trajectory

    def show(self):
        self.map.visualize(plt.gca())
        plt.show()

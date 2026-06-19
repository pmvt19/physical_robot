
from physical_robot.map_builder.map_builder import MapBuilder
from physical_robot.maps import AdvancedMap

class AdvancedMapBuilder(MapBuilder):
    def __init__(self, robot, map_resolution=10.0, manual_lidar_verification=False):
        super().__init__(robot, map_resolution, manual_lidar_verification)
        self.map: AdvancedMap = AdvancedMap(resolution=map_resolution)
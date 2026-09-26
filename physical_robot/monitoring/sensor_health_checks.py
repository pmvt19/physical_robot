from physical_robot.robot import Robot


class ExpandingList:
    def __init__(self, max_size=10):
        self.list = []
        self.max_size = max_size

    def get_max_size(self):
        return self.max_size

    def append(self, data):
        if len(self.list) == self.max_size:
            self.list.pop(0)

        self.list.append(data)

    def get_list(self):
        return self.list


    


class SensorHealthMonitoring:
    def __init__(self):
        self.robot = Robot(connection="client")

        self.lidar_latencies = ExpandingList()
        self.lidar_previous_timestamp = None

    def get_lidar_health(self):
        sensor_time = self.robot._get_single_lidar_reading()

        if self.lidar_previous_timestamp:
            latency = sensor_time - self.lidar_previous_timestamp
            self.lidar_latencies.append(latency)

        self.lidar_previous_timestamp = sensor_time

    def get_camera_health(self):
        pass

    def visualize_latencies(self):
        # TODO: Visualize with Rerun
        pass
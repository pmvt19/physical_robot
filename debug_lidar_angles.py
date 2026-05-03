from robot import Robot
import matplotlib.pyplot as plt
import numpy as np

def animate_lidar_scan(scan, lidar_data, delay=0.1):
    num_points = len(lidar_data)
    min_coords = np.min(scan, axis=0)
    max_coords = np.max(scan, axis=0)

    buffer = np.max(np.abs(scan), axis=0) * 0.1
    for i in range(num_points):
        plt.title(f"Angle: {lidar_data[i, 0]}")
        plt.scatter(0, 0, color='red')
        plt.scatter(scan[0:(i+1), 0], scan[0:(i+1), 1])
        plt.xlim(min_coords[0] - buffer[0], max_coords[0] + buffer[0])
        plt.ylim(min_coords[1] - buffer[1], max_coords[1] + buffer[1])
        plt.pause(delay)
        plt.cla()

    plt.scatter(0, 0, color='red')
    plt.scatter(scan[:, 0], scan[:, 1])
    plt.xlim(min_coords[0] - buffer[0], max_coords[0] + buffer[0])
    plt.ylim(min_coords[1] - buffer[1], max_coords[1] + buffer[1])
    plt.show()


if __name__ == '__main__':
    robot = Robot(connection='client')

    scan, lidar_data = robot.read_lidar_updated(manual_verification=False, wait_for_updated_reading=True)
    lidar_data[:, 0] = 360 - lidar_data[:, 0]
    lidar_data[:, 0] = lidar_data[:, 0] + 90
    lidar_data[:, 0] = lidar_data[:, 0] % 360
    animate_lidar_scan(scan, lidar_data)


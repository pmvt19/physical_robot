import time
import logging
import numpy as np
from rplidar import RPLidar

import redis

from physical_robot.config import config

from utils import register_logger

logger = register_logger(
    logger_name=__name__,
    log_filename="lidar_runner",
    level=logging.INFO,
    std_err=False,
)

def start_lidar():
    # Connect to Redis
    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    # Initialize Lidar Client
    lidar = RPLidar(config['physical']['lidar_port'], baudrate=460800)
    time.sleep(5)
    lidar.clean_input()
    
    info = lidar.get_info()
    print(info)

    health = lidar.get_health()
    print(health)

    try:
        for i, scan in enumerate(lidar.iter_scans()):
            st = time.time()
            
            angles = []
            dists = []
            for s in scan:
                quality, angle, distance = s
                angles.append(angle)
                dists.append(distance)

            
            angles = np.array(angles)
            dist = np.array(dists)

            lidar_output = np.stack((angles, dist), axis=1)
            redis_client.set('lidar_data', lidar_output.tobytes())
            redis_client.set('time', time.time())

            et = time.time()

            frame_time = et - st
            fps = 1 / frame_time
            logger.info(f"FPS: {fps}")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Performing cleanup...")
        # Add your cleanup or data processing logic here
        # For example, saving data to a file, closing resources, etc.
        print("Cleanup complete. Exiting.")
        # sys.exit(0)

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == "__main__":
    start_lidar()
    
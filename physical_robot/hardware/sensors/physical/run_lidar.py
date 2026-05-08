import time
import numpy as np
from rplidar import RPLidar

import redis

from config import config

def start_lidar():

    # Connect to Redis
    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    lidar = RPLidar(config['physical']['lidar_port'], baudrate=460800)
    time.sleep(5)
    lidar.clean_input()

    info = lidar.get_info()
    print(info)

    health = lidar.get_health()
    print(health)

    try:
        for i, scan in enumerate(lidar.iter_scans()):
            
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

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Performing cleanup...")
        # Add your cleanup or data processing logic here
        # For example, saving data to a file, closing resources, etc.
        print("Cleanup complete. Exiting.")
        # sys.exit(0)

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == '__main__':
    start_lidar()
    
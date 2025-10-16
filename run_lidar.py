import time
import numpy as np
from rplidar import RPLidar

import redis

def start_lidar(data):
    lidar = RPLidar('/dev/tty.usbserial-14220', baudrate=460800)
    time.sleep(5)
    lidar.clean_input()

    info = lidar.get_info()
    print(info)

    health = lidar.get_health()
    print(health)

    start_time = time.time()
    run_time = 300

    try:
        for i, scan in enumerate(lidar.iter_scans()):
            
            angles = []
            dists = []
            for s in scan:
                quality, angle, distance = s
                angles.append(angle)
                dists.append(distance)

            
            angles = np.array(angles)
            dist = np.array(dists) #/ 10.0

            lidar_output = np.stack((angles, dist), axis=1)
            # np.save('lidar_data/scan.npy', lidar_output)
            data['lidar'] = lidar_output

            # if time.time() > start_time + run_time:
            #     break

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Performing cleanup...")
        # Add your cleanup or data processing logic here
        # For example, saving data to a file, closing resources, etc.
        print("Cleanup complete. Exiting.")
        # sys.exit(0)

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

def start_lidar_local():

    # Connect to Redis
    redis_client = redis.Redis(host='localhost', port=6379, db=0)

    lidar = RPLidar('/dev/ttyUSB1', baudrate=460800)
    # lidar.stop()
    # lidar.stop_motor()


    # time.sleep(5)
    # lidar.clean_input()
    # time.sleep(5)

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
            dist = np.array(dists) #/ 10.0

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
    start_lidar_local()
    
import time
import numpy as np
import rerun as rr
from rplidar import RPLidar

from config import config

def start_lidar():
    lidar = RPLidar(config['physical']['lidar_port'], baudrate=460800)
    time.sleep(5)
    lidar.clean_input()

    info = lidar.get_info()
    print(info)
    health = lidar.get_health()
    print(health)

    start_time = time.time()

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

            cos = np.cos(angles)
            sin = np.sin(angles)

            x_coords = dist * cos
            y_coords = dist * sin
            z_coords = np.ones_like(x_coords)
            coords = np.stack((x_coords, y_coords, z_coords), axis=1)

            rr.set_time("time", duration=time.time()-start_time)
            rr.log("points", rr.Points3D(coords))
            rr.log("points v2", rr.Points3D([[[0.0,0.0,0.0]]], colors=[0, 255, 0], radii=10.0))

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
    
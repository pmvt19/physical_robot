from physical_robot.robot import Robot

import rerun as rr
import numpy as np
import time

if __name__ == '__main__':
    run_time = 300

    robot = Robot(connection='client')

    start_time = time.time()

    rr.init("3d points", spawn=True)

    while True:
        coords, raw_lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)
        rr.set_time("time", duration=time.time()-start_time)
        coords = np.stack((coords[:, 0], coords[:, 1], coords[:, 2]), axis=1)
        rr.log("points", rr.Points3D(coords))
        rr.log("points v2", rr.Points3D([[[0.0,0.0,0.0]]], colors=[0, 255, 0], radii=10.0))

        if time.time() > start_time + run_time:
            break

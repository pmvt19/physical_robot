import time

import numpy as np
import rerun as rr

from physical_robot.robot import Robot

if __name__ == "__main__":
    run_time = 300

    robot = Robot(connection="client")

    start_time = time.time()

    rr.init("3d points", spawn=True)

    while True:
        coords, colors = robot.read_point_cloud()
        rr.set_time("time", duration=time.time() - start_time)
        coords = np.copy(coords)
        coords = np.stack((coords[:, 0], coords[:, 1], coords[:, 2]), axis=1)
        rr.log("points", rr.Points3D(coords, colors=colors))

        if time.time() > start_time + run_time:
            break

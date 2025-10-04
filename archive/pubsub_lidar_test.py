import time
import numpy as np
from rplidar import RPLidar
import rerun as rr
from robot import Robot


start_time = time.time()
run_time = 300
rerun_instance_created = False
# while time.time() < start_time + run_time:

robot = Robot()

while True:
    if not rerun_instance_created:
        rr.init("3d points", spawn=True)
        rerun_instance_created = True
    
    coords = robot.read_lidar()
    rr.set_time("time", duration=time.time()-start_time)
    rr.log("points", rr.Points3D(coords))
    rr.log("points v2", rr.Points3D([[[0.0,0.0,0.0]]], colors=[0, 255, 0], radii=0.1))

    if time.time() > start_time + run_time:
        break
import time
import numpy as np
from rplidar import RPLidar
import rerun as rr



lidar = RPLidar('/dev/tty.usbserial-14220', baudrate=460800)
time.sleep(5)
lidar.clean_input()

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)



# for i, scan in enumerate(lidar.iter_scans()):
#     print('%d: Got %d measurments' % (i, len(scan)))
#     if i > 10:
#         break

start_time = time.time()
run_time = 300
rerun_instance_created = False
# while time.time() < start_time + run_time:
try:
    for i, scan in enumerate(lidar.iter_scans()):

        if not rerun_instance_created:
            rr.init("3d points", spawn=True)
            rerun_instance_created = True
        
        angles = []
        dists = []
        for s in scan:
            quality, angle, distance = s
            angles.append(angle)
            dists.append(distance)

        
        angles = np.array(angles)
        dist = np.array(dists) / 10.0

        rad_angles = (np.pi / 180.0) * angles

        cos = np.cos(rad_angles)
        sin = np.sin(rad_angles)

        x_coords = dist * cos
        y_coords = dist * sin
        z_coords = np.zeros_like(x_coords)

        rr.set_time("time", duration=time.time()-start_time)
        coords = np.stack((x_coords, y_coords, z_coords), axis=1)
        rr.log("points", rr.Points3D(coords))
        rr.log("points v2", rr.Points3D([[[0.0,0.0,0.0]]], colors=[0, 255, 0], radii=0.1))

        if time.time() > start_time + run_time:
            break
except KeyboardInterrupt:
    print("\nCtrl+C detected. Performing cleanup...")
    # Add your cleanup or data processing logic here
    # For example, saving data to a file, closing resources, etc.
    print("Cleanup complete. Exiting.")
    # sys.exit(0)


lidar.stop()
lidar.stop_motor()
lidar.disconnect()
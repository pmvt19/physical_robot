from physical_robot.robot import Robot

import rerun as rr
import numpy as np
import time

if __name__ == '__main__':
    run_time = 300
    # rr.init("IMU Data", spawn=True)
    # time.sleep(10)
    rr.init("IMU Data", spawn=True, init_logging=True)
    # rr.connect()

    robot = Robot(connection='client')

    start_time = time.time()
    
    try:

        while True:
            accel_data = [np.random.random(), np.random.random(), np.random.random()]
            gyro_data = None
            accel_data, gyro_data = robot.read_imu()

            print("Accel Data,", accel_data)
            print("Gyro Data", gyro_data)

            # rr.set_time("time", time.time())
            rr.set_time("time", duration=time.time()-start_time)
            rr.log("accelerometer x", rr.Scalars(float(accel_data[0])))
            rr.log("accelerometer y", rr.Scalars(accel_data[1]))
            rr.log("accelerometer z", rr.Scalars(accel_data[2]))

            rr.log("gyroscope x", rr.Scalars(gyro_data[0]))
            rr.log("gyroscope y", rr.Scalars(gyro_data[1]))
            rr.log("gyroscope z", rr.Scalars(gyro_data[2]))


            if time.time() > start_time + run_time:
                break
            # time.sleep(0.1)
    except KeyboardInterrupt:
        # rr.flush()
        # rr.close()
        exit()

import numpy as np

from physical_robot.robot.robot import Robot

robot = Robot(connection='client')

idx = 0

state = np.array([0.0, 0.0, 0.0])

motion_method = "rotation_and_translation"

while True:

    # Read Lidar 
    coords, lidar_data = robot.read_lidar_updated(manual_verification=True, wait_for_updated_reading=True)

    # TODO: Save Lidar and State Data HERE
    np.save(f"./test_data/icp/icp_sample_data_{motion_method}_{idx}.npy", coords)
    np.save(f"./test_data/icp/state_{motion_method}_{idx}.npy", state)

    idx += 1

    # Move Robot
    motion_command = robot.request_motion_command_from_user()
    if motion_command[0] == '': # No Motion Command
        break
    m, state = robot.command_motion_and_predict_state(state, motion_command)

import numpy as np

map_size = (100, 100)
res = 0.1

om = np.zeros((int(map_size[0]/res), int(map_size[1]/res)))

# Robot Algorithm Outline

# Set of motion commands: U

# for u in U:
#   motion = robot.move(u)
#   predicted_state = robot.predict_state(motion)
#   lidar_reading = robot.read_lidar()
#   updated_state = map.localize_robot(predicted_state, lidar_reading)
#   final_state = ?? (Some weighted average of predicted vs updated_state?)


# Map algorithm outline 
#  --- Ideally, we reuse the occupancy_map in pmpl
#  --- Or, rewrite that map entirely and replace the existing implementation
#  --- The current map does not support the robot being rotated

# map.localize_robot(predicted_state, lidar_reading):
#   T = icp(target=map_point_cloud, source=lidar_reading)
#   extracted_state = extract_state(T)
#   -- IMPORTANT --
#   add_points_to_map(lidar_reading) # Need to expand the map if possible
#   return extracted_state




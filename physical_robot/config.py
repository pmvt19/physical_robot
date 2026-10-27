# class RobotConfig():
#     def __init__(self):
#         pass

parameters_dict = {
    'simulated' : {
        'lidar_map_path' : None
    },
    'client' : {
        'channel_address': '192.168.12.155:50051'
    },
    'physical' : {
        'lidar_port' : '/dev/ttyUSB0', # Linux
        'dxl_motor_port' : '/dev/ttyUSB1', # Linux
        'redis_host': 'localhost',
        'redis_port': 6379
    }
}
connection_type = 'client'

config = {
    'connection_type': connection_type,
    connection_type : parameters_dict[connection_type]
}

scene = 'tmp'

# Robot Dimensions
WHEEL_DIAMETER = 66.5 # cm
WHEEL_RADIUS = WHEEL_DIAMETER / 2 # cm
ROBOT_WHEELBASE_LENGTH = 210 # cm
ROBOT_DIAMETER = ROBOT_WHEELBASE_LENGTH # cm
ROBOT_RADIUS = ROBOT_DIAMETER / 2 # cm

# Motor Constants/Configurations
DYNAMIXEL_PULSE_PER_REVOLUTION = 4096
DYNAMIXEL_BAUD_RATE = 57600
DYNAMIXEL_PROTOCOL = 2.0
RPM_CONVERSION = 0.229 # TODO: TBD

# Simulated Lidar Constants
SIMULATED_LIDAR_RESOLUTION = 360 # Degrees
SIMULATED_LIDAR_MAX_LENGTH = 12000 # mm

# Lidar Scanner
LIDAR_MIN_POINT_DIST = 90 # mm
LIDAR_AUTO_RETRY_THRESHOLD = 400 # num points
LIDAR_BAUD_RATE = 460800
LIDAR_STARTUP_DELAY = 5 # seconds

# Map Constants
DEFAULT_RESOLUTION = 10.0 # mm
DEFAULT_MAP_SIZE = (20000, 20000) # mm x mm
INFLATION_RADIUS = 210 # mm (Same as ROBOT_DIAMETER)
EXPANSION_BUFFER = 0.3 # TODO: TBD

# Advanced Map
OCCUPANCY_THRESHOLD = 0.5 # proportion

FRONTIER_SMOOTHING = 1.0 # TODO: TBD
FRONTIER_MIN_CLUSTER_SIZE = 100 # TODO: TBD (CELLs?)
FRONTIER_MIN_OBST_DIST = 400 # mm

# Particle Filter
RESAMPLE_COUNT = 1000

# ICP
INIT_INLIER_DIST_THRESHOLD = 200 # mm
INLIDER_DIST_THRESHOLD = 100 # mm
INLIER_RATIO = 0.999 # proportion
MAX_ITERATIONS = 15

# Motion Planner
PATH_LOOKAHEAD = 4
GOAL_DIST_THRESHOLD = 100 # mm

EDGE_VALIDITY_DIST = 200.0 # TODO: TBD?
COLLISION_CHECK_BATCH_SIZE = 1000

# Image Segmenter
MIN_SEGMENT_SIZE = 0.01

# VLM
VLM_TEMPERATURE = 0.5


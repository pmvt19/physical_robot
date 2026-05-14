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


from dxl_controller import *
import numpy as np
import time
from utils import register_logger

from threading import Lock

logger = register_logger(logger_name=__name__, log_filename='robot_interface', level=logging.DEBUG)

class MockLock():
    def __init__(self):
        pass

    def acquire(self):
        pass

    def release(self):
        pass

class RobotInterface():
    def __init__(self, controller):
        self.controller : DynamixelController = controller

        self.linear_velocity = 10
        self.controller_lock = Lock()

        # self.controller.set_torque(1, TORQUE_DISABLE)
        # self.controller.set_torque(2, TORQUE_DISABLE)
        self.set_torque(TORQUE_DISABLE)

        # self.controller.set_operating_mode(1, VELOCITY_CONTROL_MODE)
        # self.controller.set_operating_mode(2, VELOCITY_CONTROL_MODE)

        # self.controller.set_operating_mode(1, EXTENDED_POSITION_CONTROL_MODE)
        # self.controller.set_operating_mode(2, EXTENDED_POSITION_CONTROL_MODE)
        self.set_mode(EXTENDED_POSITION_CONTROL_MODE)

        # self.controller.set_torque(1, TORQUE_ENABLE)
        # self.controller.set_torque(2, TORQUE_ENABLE)
        self.set_torque(TORQUE_ENABLE)

        self.r = 66.5 / 2
        self.L = 210

        self.pulse_per_rev = 4096

        self.before_motion_positions = []
        self.after_motion_positions = []

    def set_torque(self, torque):
        self.controller_lock.acquire()

        self.controller.set_torque(1, torque)
        self.controller.set_torque(2, torque)

        self.controller_lock.release()

    def set_mode(self, mode):
        self.controller_lock.acquire()

        self.controller.set_operating_mode(1, mode)
        self.controller.set_operating_mode(2, mode)

        self.controller_lock.release()
    
    def set_profile_velocity(self, velocity=100):
        self.controller_lock.acquire()

        self.controller.set_profile_velocity(id=1, velocity=velocity)
        self.controller.set_profile_velocity(id=2, velocity=velocity)

        self.controller_lock.release()

    def get_motor_velocity(self):
        self.controller_lock.acquire()

        v1 = self.controller.get_velocity(id=1)
        v2 = self.controller.get_velocity(id=1)

        self.controller_lock.release()
        return np.array([v1, v2])
    
    def set_motor_positions(self, positions):
        self.controller_lock.acquire()

        self.controller.set_position(id=1, position=positions[0])
        self.controller.set_position(id=2, position=positions[1])

        self.controller_lock.release()
    
    def get_motor_positions(self):
        self.controller_lock.acquire()

        motor_id_1_pos = self.controller.get_position(id=1)
        motor_id_2_pos = self.controller.get_position(id=2)
        self.controller_lock.release()

        return np.array([motor_id_1_pos, motor_id_2_pos], dtype=np.int64)
        
    def move_forward(self):
        self.controller_lock.acquire()

        self.controller.set_velocity(id=1, velocity_rpm=self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=-self.linear_velocity)

        self.controller_lock.release()

    def move_backward(self):
        self.controller_lock.acquire()

        self.controller.set_velocity(id=1, velocity_rpm=-self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=self.linear_velocity)

        self.controller_lock.release()

    def rotate_right(self):
        self.controller_lock.acquire()

        self.controller.set_velocity(id=1, velocity_rpm=self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=self.linear_velocity)

        self.controller_lock.release()
    
    def rotate_left(self):
        self.controller_lock.acquire()

        self.controller.set_velocity(id=1, velocity_rpm=-self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=-self.linear_velocity)

        self.controller_lock.release()
    
    def stop_motion(self):
        self.controller_lock.acquire()

        self.controller.set_velocity(id=1, velocity_rpm=0)
        self.controller.set_velocity(id=2, velocity_rpm=0)

        self.controller_lock.release()

    



if __name__ == '__main__':
    controller = DynamixelController(device_name='/dev/tty.usbserial-FTAKRMAJ', motor_ids=[1, 2])
    ri = RobotInterface(controller=controller)

    
    

    


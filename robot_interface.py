from dxl_controller import *
import numpy as np
import time

class RobotInterface():
    def __init__(self, controller):
        self.controller : DynamixelController = controller

        self.linear_velocity = 10

        self.controller.set_torque(1, TORQUE_DISABLE)
        self.controller.set_torque(2, TORQUE_DISABLE)

        self.controller.set_operating_mode(1, VELOCITY_CONTROL_MODE)
        self.controller.set_operating_mode(2, VELOCITY_CONTROL_MODE)

        self.controller.set_torque(1, TORQUE_ENABLE)
        self.controller.set_torque(2, TORQUE_ENABLE)

        self.r = 66.5 / 2
        self.L = 210
        
    def move_forward(self):
        self.controller.set_velocity(id=1, velocity_rpm=self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=-self.linear_velocity)

    def move_backward(self):
        self.controller.set_velocity(id=1, velocity_rpm=-self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=self.linear_velocity)

    def rotate_right(self):
        self.controller.set_velocity(id=1, velocity_rpm=self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=self.linear_velocity)
    
    def rotate_left(self):
        self.controller.set_velocity(id=1, velocity_rpm=-self.linear_velocity)
        self.controller.set_velocity(id=2, velocity_rpm=-self.linear_velocity)
    
    def stop_motion(self):
        self.controller.set_velocity(id=1, velocity_rpm=0)
        self.controller.set_velocity(id=2, velocity_rpm=0)

    def rotate_deg(self, deg=np.pi/2):
        robot_rpm = self.r * self.linear_velocity / (self.L / 2)

        rotation_frac = deg / (2 * np.pi)

        # rotation_frac

        rotational_speed_sec_per_rot = (1 / robot_rpm) * 60

        rotation_seconds = rotational_speed_sec_per_rot * rotation_frac

        # self.rotate_right()
        self.rotate_left()
        time.sleep(rotation_seconds)
        self.stop_motion()

    def move_mm(self, mm=100):
        dist_per_rev = self.r * 2 * np.pi
        required_revs = mm / dist_per_rev

        print(required_revs)

        linear_seconds = required_revs / self.linear_velocity * 60 # RPS

        # self.move_forward()
        self.move_backward()
        time.sleep(linear_seconds)
        self.stop_motion()

    def get_motor_positions(self):
        motor_id_1_pos = self.controller.get_position(id=1)
        motor_id_2_pos = self.controller.get_position(id=2)
        return [motor_id_1_pos, motor_id_2_pos]



    # Wheel Diameter (r) is 66.5 mm
    # Wheel Base (L) is 105*2 mm = 210 mm

def rotational_main(controller, ri):
    init_motor_pos = ri.get_motor_positions()
    print(init_motor_pos)
    ri.rotate_deg(deg=np.pi/2)
    final_motor_pos = ri.get_motor_positions()
    print(final_motor_pos)

    init_motor_pos = np.array(init_motor_pos)
    final_motor_pos = np.array(final_motor_pos)

    motor_pos_diff = final_motor_pos - init_motor_pos
    print(motor_pos_diff)

    # motor_pos_diff = np.array([-62379, -62388])
    rev_diff = motor_pos_diff / 4096
    print(rev_diff)
    # rot_diff = ri.r * rev_diff / (ri.L / 2)
    rot_diff = 2 * ri.r * rev_diff / (ri.L)

    print(rev_diff, rot_diff)

    rad_rotated = rot_diff * (2*np.pi)

    print(rad_rotated)

def linear_main(controller, ri):
    init_motor_pos = ri.get_motor_positions()
    print(init_motor_pos)
    ri.move_mm()
    # exit()
    final_motor_pos = ri.get_motor_positions()
    print(final_motor_pos)

    init_motor_pos = np.array(init_motor_pos)
    final_motor_pos = np.array(final_motor_pos)

    diff = final_motor_pos - init_motor_pos

    print(diff)

    revs = diff / 4096

    print(revs)

    cir = np.pi * 66.5

    dists = revs * cir 

    print(dists)


if __name__ == '__main__':
    controller = DynamixelController(device_name='/dev/tty.usbserial-FTAKRMAJ', motor_ids=[1, 2])
    ri = RobotInterface(controller=controller)
    # linear_main(controller, ri)
    rotational_main(controller, ri)

    # st = time.time()
    # ri.move_forward()
    # while (time.time() - st) < 5:
    #     print(ri.get_motor_positions())

    
    

    


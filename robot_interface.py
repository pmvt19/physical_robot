from dxl_controller import *
import numpy as np
import time

class RobotInterface():
    def __init__(self, controller):
        self.controller : DynamixelController = controller

        self.linear_velocity = 20

        self.controller.set_torque(1, TORQUE_DISABLE)
        self.controller.set_torque(2, TORQUE_DISABLE)

        self.controller.set_operating_mode(1, VELOCITY_CONTROL_MODE)
        self.controller.set_operating_mode(2, VELOCITY_CONTROL_MODE)

        self.controller.set_torque(1, TORQUE_ENABLE)
        self.controller.set_torque(2, TORQUE_ENABLE)

        self.r = 66.5 / 2
        self.L = 210

        self.before_motion_positions = []
        self.after_motion_positions = []
        
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
        init_motor_pos = ri.get_motor_positions()

        robot_rpm = self.r * self.linear_velocity / (self.L / 2)

        rotation_frac = deg / (2 * np.pi)

        # rotation_frac

        rotational_speed_sec_per_rot = (1 / robot_rpm) * 60

        rotation_seconds = rotational_speed_sec_per_rot * rotation_frac

        # self.rotate_right()
        self.rotate_left()
        time.sleep(rotation_seconds)
        self.stop_motion()

        final_motor_pos = ri.get_motor_positions()

        return self.compute_rotation_motion(init_motor_pos, final_motor_pos)

    def move_mm(self, mm=100):
        init_motor_pos = ri.get_motor_positions()

        dist_per_rev = self.r * 2 * np.pi
        required_revs = mm / dist_per_rev

        linear_seconds = required_revs / self.linear_velocity * 60 # RPS

        self.move_forward()
        # self.move_backward()
        time.sleep(linear_seconds)
        self.stop_motion()

        final_motor_pos = ri.get_motor_positions()

        return self.compute_linear_motion(init_motor_pos, final_motor_pos)

    def compute_linear_motion(self, init_mp, final_mp):
        init_mp = np.array(init_mp)
        final_mp = np.array(final_mp)
        print(init_mp, final_mp)

        diff = final_mp - init_mp

        revs = diff / 4096

        cir = np.pi * 66.5

        dists = revs * cir 
        return dists
    
    def compute_rotation_motion(self, init_mp, final_mp):
        init_mp = np.array(init_mp)
        final_mp = np.array(final_mp)
        print(init_mp, final_mp)

        motor_pos_diff = final_mp - init_mp

        rev_diff = motor_pos_diff / 4096
        # rot_diff = ri.r * rev_diff / (ri.L / 2)
        rot_diff = 2 * ri.r * rev_diff / (ri.L)

        rad_rotated = rot_diff * (2*np.pi)

        return rad_rotated


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
    # rotational_main(controller, ri)

    linear_movement_1 = ri.move_mm(200)
    input()
    rotation_1 = ri.rotate_deg(deg=np.pi/2)
    input()
    linear_movement_2 = ri.move_mm(450)
    input()
    rotation_2 = ri.rotate_deg(deg=np.pi/2)
    input()
    linear_movement_3 = ri.move_mm(200)
    input()

    print(linear_movement_1)
    print(rotation_1)
    print(linear_movement_2)
    print(rotation_2)
    print(linear_movement_3)
    # st = time.time()
    # ri.move_forward()
    # while (time.time() - st) < 5:
    #     print(ri.get_motor_positions())

    
    

    


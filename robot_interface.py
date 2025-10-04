from dxl_controller import *
import numpy as np
import time
from utils import register_logger

logger = register_logger(logger_name=__name__, log_filename='robot_interface', level=logging.DEBUG)

class RobotInterface():
    def __init__(self, controller):
        self.controller : DynamixelController = controller

        self.linear_velocity = 10

        self.controller.set_torque(1, TORQUE_DISABLE)
        self.controller.set_torque(2, TORQUE_DISABLE)

        # self.controller.set_operating_mode(1, VELOCITY_CONTROL_MODE)
        # self.controller.set_operating_mode(2, VELOCITY_CONTROL_MODE)

        self.controller.set_operating_mode(1, EXTENDED_POSITION_CONTROL_MODE)
        self.controller.set_operating_mode(2, EXTENDED_POSITION_CONTROL_MODE)

        self.controller.set_torque(1, TORQUE_ENABLE)
        self.controller.set_torque(2, TORQUE_ENABLE)

        self.r = 66.5 / 2
        self.L = 210

        self.before_motion_positions = []
        self.after_motion_positions = []

    def set_torque(self, torque):
        self.controller.set_torque(1, torque)
        self.controller.set_torque(2, torque)

    def set_mode(self, mode):
        raise NotImplementedError
    
    def set_profile_velocity(self, velocity=100):
        self.controller.set_profile_velocity(id=1, velocity=velocity)
        self.controller.set_profile_velocity(id=2, velocity=velocity)

    def get_motor_velocity(self):
        v1 = self.controller.get_velocity(id=1)
        v2 = self.controller.get_velocity(id=1)
        return np.array([v1, v2])
        
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
        init_motor_pos = self.get_motor_positions()

        robot_rpm = self.r * self.linear_velocity / (self.L / 2)

        rotation_frac = deg / (2 * np.pi)

        # rotation_frac

        rotational_speed_sec_per_rot = (1 / robot_rpm) * 60

        rotation_seconds = rotational_speed_sec_per_rot * rotation_frac

        # self.rotate_right()
        self.rotate_left()
        time.sleep(rotation_seconds)
        self.stop_motion()

        final_motor_pos = self.get_motor_positions()

        return self.compute_rotation_motion(init_motor_pos, final_motor_pos)

    def move_mm(self, mm=100):
        init_motor_pos = self.get_motor_positions()

        dist_per_rev = self.r * 2 * np.pi
        required_revs = mm / dist_per_rev

        linear_seconds = required_revs / self.linear_velocity * 60 # RPS

        self.move_forward()
        # self.move_backward()
        time.sleep(linear_seconds)
        self.stop_motion()

        final_motor_pos = self.get_motor_positions()

        return self.compute_linear_motion(init_motor_pos, final_motor_pos)
    
    def move_dist(self, mm=100):
        self.set_torque(TORQUE_DISABLE)
        init_motor_pos = self.get_motor_positions()
        self.set_torque(TORQUE_ENABLE)
        print(f"Initial Motor Positions: {init_motor_pos}")
        logger.debug(f"Initial Motor Positions: {init_motor_pos}")

        cir = self.r * 2 * np.pi

        required_revs = mm / cir

        req_pulse = int(required_revs * 4096)

        desired_motor_pos_1 = init_motor_pos[0] + req_pulse
        desired_motor_pos_2 = init_motor_pos[1] - req_pulse
        print(f"Desired Motor Positions: {[desired_motor_pos_1, desired_motor_pos_2]}")


        self.controller.set_position(id=1, position=desired_motor_pos_1)
        self.controller.set_position(id=2, position=desired_motor_pos_2)

        while np.sum(self.get_motor_velocity()) > 0:
            continue
        
        final_motor_pos = self.get_motor_positions()


        if desired_motor_pos_1 < 0:
            print("Deflating final motor position 1")
            final_motor_pos[0] -= self.controller.max_motor_position

        if desired_motor_pos_2 < 0:
            print("Deflating final motor position 2")
            final_motor_pos[1] -= self.controller.max_motor_position

        print(f"Final Motor Positions: {final_motor_pos}")
        return self.compute_linear_motion(init_motor_pos, final_motor_pos)
    
    def rotate_rad(self, rad=np.pi/2):
        init_motor_pos = self.get_motor_positions()
        print(f"Initial Motor Positions: {init_motor_pos}")

        cir = self.r * 2 * np.pi
        body_cir = 2 * np.pi * (self.L / 2)

        rotation_percentage = rad / (2 * np.pi)

        wheel_travel_dist = body_cir * rotation_percentage

        required_revs = wheel_travel_dist / cir

        req_pulse = int(required_revs * 4096)

        desired_motor_pos_1 = init_motor_pos[0] + req_pulse
        desired_motor_pos_2 = init_motor_pos[1] + req_pulse
        print(f"Desired Motor Positions: {[desired_motor_pos_1, desired_motor_pos_2]}")

        self.controller.set_position(id=1, position=desired_motor_pos_1)
        self.controller.set_position(id=2, position=desired_motor_pos_2)

        while np.sum(self.get_motor_velocity()) > 0:
            continue
        
        print(f"Before Update Init: {init_motor_pos}")

        if desired_motor_pos_1 < 0:
            print("Deflating final motor position 1")
            final_motor_pos[0] -= self.controller.max_motor_position
            
        if desired_motor_pos_2 < 0:
            print("Deflating final motor position 2")
            final_motor_pos[1] -= self.controller.max_motor_position
        
        print(f"After Update Init: {init_motor_pos}")
        print(f"Data Type init: {init_motor_pos.dtype}")

        desired_motor_pos_1 = desired_motor_pos_1 % self.controller.max_motor_position
        desired_motor_pos_2 = desired_motor_pos_2 % self.controller.max_motor_position

        print(f"Updated Desired Motor Positions: {[desired_motor_pos_1, desired_motor_pos_2]}")

        final_motor_pos = self.get_motor_positions()
        print(f"Final Motor Positions: {final_motor_pos}")
        return self.compute_rotation_motion(init_motor_pos, final_motor_pos)
    

    def compute_linear_motion(self, init_mp, final_mp):
        init_mp = np.array(init_mp)
        final_mp = np.array(final_mp)
        print(init_mp, final_mp)

        diff = final_mp - init_mp
        print(f"Motor Differentials: {diff}")

        revs = diff / 4096

        cir = np.pi * 66.5

        dists = revs * cir 
        return dists
    
    def compute_rotation_motion(self, init_mp, final_mp):
        init_mp = np.array(init_mp)
        final_mp = np.array(final_mp)
        print(init_mp, final_mp)

        motor_pos_diff = final_mp - init_mp
        print(f"Motor Differentials: {motor_pos_diff}")

        rev_diff = motor_pos_diff / 4096
        # rot_diff = ri.r * rev_diff / (ri.L / 2)
        rot_diff = 2 * self.r * rev_diff / (self.L)

        rad_rotated = rot_diff * (2*np.pi)

        return rad_rotated


    def get_motor_positions(self):
        motor_id_1_pos = self.controller.get_position(id=1)
        motor_id_2_pos = self.controller.get_position(id=2)
        # return np.array([motor_id_1_pos, motor_id_2_pos], dtype=np.uint64)
        return np.array([motor_id_1_pos, motor_id_2_pos], dtype=np.int64)



    # Wheel Diameter (r) is 66.5 mm
    # Wheel Base (L) is 105*2 mm = 210 mm

    # def predict_state(self, state, motion, motion_type='linear'):
    #     motion = np.abs(motion) # This should be directional and not a magnitude
    #     # avg_motion = np.mean(motion)
    #     avg_motion = np.min(motion)

    #     x, y, theta = state

    #     if motion_type == 'linear':

    #         direction_vector = np.array([np.cos(theta), np.sin(theta), 0.0])
    #         dx_state = direction_vector * avg_motion
    #         updated_state = state + dx_state

    #     elif motion_type == 'angular':
    #         dx_state = np.array([0.0, 0.0, avg_motion])
    #         updated_state = state + dx_state
    #     else:
    #         raise NotImplementedError

    #     return updated_state

    def predict_state(self, state, motor_position_differential):
        signs = np.sign(motor_position_differential)

        
        abs_motion = np.abs(motor_position_differential)
        if signs[0] == -1 and signs[1] == -1:
            # Turn In-Place Left
            avg_motion = np.mean(abs_motion)
            motion_type = 'angular'
        elif signs[0] == 1 and signs[1] == 1:
            # Turn In-Place Right
            avg_motion = -1 * np.mean(abs_motion)
            motion_type = 'angular'
        elif signs[0] == 1 and signs[1] == -1:
            # Forward
            avg_motion = np.mean(abs_motion)
            motion_type = 'linear'
        elif signs[0] == -1 and signs[1] == 1:
            # Backward?
            avg_motion = -1 * np.mean(abs_motion)
            motion_type = 'linear'
        else:
            print("Ensure the U2D2 PowerBoard is On")
            raise NotImplementedError

        x, y, theta = state

        if motion_type == 'linear':

            direction_vector = np.array([np.cos(theta), np.sin(theta), 0.0])
            dx_state = direction_vector * avg_motion
            updated_state = state + dx_state

        elif motion_type == 'angular':
            dx_state = np.array([0.0, 0.0, avg_motion])
            updated_state = state + dx_state
        else:
            raise NotImplementedError

        return updated_state


if __name__ == '__main__':
    controller = DynamixelController(device_name='/dev/tty.usbserial-FTAKRMAJ', motor_ids=[1, 2])
    ri = RobotInterface(controller=controller)
    # linear_main(controller, ri)
    # rotational_main(controller, ri)

    # Real Life Env Testing
    # linear_movement_1 = ri.move_mm(200)
    # input()
    # rotation_1 = ri.rotate_deg(deg=np.pi/2)
    # input()
    # linear_movement_2 = ri.move_mm(450)
    # input()
    # rotation_2 = ri.rotate_deg(deg=np.pi/2)
    # input()
    # linear_movement_3 = ri.move_mm(200)
    # input()

    # print(linear_movement_1)
    # print(rotation_1)
    # print(linear_movement_2)
    # print(rotation_2)
    # print(linear_movement_3)

    # Testing Computed Motor Position for Distances (Rather than Timing)
    # ri.set_profile_velocity()
    # move = ri.move_dist(150)
    # move = ri.rotate_rad()
    # print(move)

    # Testing Motion and Predicted State
    # ri.set_profile_velocity()
    # state = np.array([0.0, 0.0, 0.0])
    # print(f"State: {state}")
    # m1 = ri.move_dist(300)
    # print(f"Motion: {m1}")
    # state = ri.predict_state(state, m1, 'linear')
    # print(f"State: {state}")
    # m2 = ri.rotate_rad(np.pi/4)
    # print(f"Motion: {m2}")
    # state = ri.predict_state(state, m2, 'angular')
    # print(f"State: {state}")
    # m3 = ri.move_dist(400)
    # print(f"Motion: {m3}")
    # state = ri.predict_state(state, m3, 'linear')
    # print(f"State: {state}")

    # Test Backwards Movement and CCW Turns
    ri.set_profile_velocity()
    state = np.array([0.0, 0.0, 0.0])
    print(f"State: {state}")
    m1 = ri.move_dist(200)
    print(f"Motion: {m1}")
    state = ri.predict_state(state, m1)
    print(f"State: {state}")
    m2 = ri.rotate_rad(-np.pi/4)
    # m2 = ri.move_dist(-200)
    print(f"Motion: {m2}")
    state = ri.predict_state(state, m2)
    print(f"State: {state}")
    m3 = ri.move_dist(200)
    print(f"Motion: {m3}")
    state = ri.predict_state(state, m3)
    print(f"State: {state}")
    m4 = ri.rotate_rad(np.pi/4)
    print(f"Motion: {m4}")
    state = ri.predict_state(state, m4)
    print(f"State: {state}")

    print("Final State", np.round(state, 2))

    # st = time.time()
    # ri.move_forward()
    # while (time.time() - st) < 5:
    #     print(ri.get_motor_positions())

    
    

    


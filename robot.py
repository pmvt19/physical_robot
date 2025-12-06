
import time
import copy
import redis
import rerun as rr
import numpy as np
import matplotlib.pyplot as plt
import logging
import threading

import grpc
import generated.robot_data_pb2 as pb2
import generated.robot_data_pb2_grpc as pb2_grpc

from shapely import Point, Polygon, affinity

from config import config
from dxl_controller import DynamixelController, TORQUE_ENABLE, TORQUE_DISABLE
from robot_interface import RobotInterface
from simulate_lidar import SimulatedLidar
from utils import create_rectangle_geometry, point_segment_distance, point_to_points_distance, register_logger

logger = register_logger(logger_name=__name__, log_filename='robot', level=logging.INFO, std_err=False)

class Robot():
    def __init__(self, connection='simulated'):
        self.connection = connection
        
        # TODO: Do some design work and see if this is necessary, I'm leaning to not having this, (It's not used internally)
        # TODO: This might be used internally if lidar is simulated as well so might be worth it to keep

        # Initialize Current State (Starts at [x=0.0, y=0.0, theta=0.0])
        self.state = np.array([0.0, 0.0, 0.0])

        # Robot Specific Variables:
        # self.r -> move to Robot as self.wheel_radius
        # self.L -> move to Robot as self.wheelbase_length

        self.wheel_radius = (66.5/2)
        self.wheel_circumference = 2 * self.wheel_radius * np.pi
        self.wheelbase_length = 210
        self.robot_radius = self.wheelbase_length / 2

        self.guard_active_motion = False

        # TODO: Remove after successful rollout (Use self.wheel_radius and self.wheelbase_length)
        self.r = self.wheel_radius
        self.L = self.wheelbase_length

        # Thresholds
        self.active_distance_threshold = 300
        self.local_planner_distance_threshold = 400

        # If Robot is not required to be tied to the physical robot, don't initialize the controllers
        if self.connection == 'simulated':
            self.const_reference_map_for_lidar = None
            self.simulated_lidar = SimulatedLidar(self.const_reference_map_for_lidar, angular_resolution=360, max_dist=12000) # TODO: Check units for max_dist
        elif self.connection == 'client':
            # TODO: Clean After Successful Release
            # channel = grpc.insecure_channel('192.168.12.155:50051')
            channel = grpc.insecure_channel(config['client']['channel_address'])
            self.stub = pb2_grpc.RobotServerStub(channel)
            print("Motor Logs will appear in the machine where the Robot Server is run")
        elif self.connection == 'physical':
            # Initialize Classes For Motor Control

            # TODO: Clean After Successful Release
            # self.controller = DynamixelController(device_name=device_name, motor_ids=[1, 2])
            # self.ri = RobotInterface(controller=self.controller)

            self.controller = DynamixelController(device_name=config['physical']['dxl_motor_port'], motor_ids=[1, 2])
            self.ri = RobotInterface(controller=self.controller)
            self.ri.set_profile_velocity()

            # Connect to Redis Server for Publishing Lidar Data
            # self.redis_client = redis.Redis(host='localhost', port=6379, db=0)

            self.redis_client = redis.Redis(host=config['physical']['redis_host'], port=config['physical']['redis_port'], db=0)
        else:
            raise NotImplementedError

    def run_keyboard_control(self):
        """
        See if we can run keyboard control from here if the robot is not simulated
        """
        pass

    ### ------- Reading Sensors ------- ###
    
    def _get_single_lidar_reading(self, wait_for_updated_reading):
        if self.connection == 'simulated':
            # DO SOMETHING
            raise NotImplementedError
        else:
            if self.connection == 'client':
                # Make RPC Call here 
                request_ack = pb2.Acknowledge(
                    success=True,
                    message="Client is ready for data!"
                )
                lidar_data = self.stub.GetLatestLidarData(request_ack)

                init_time = lidar_data.timestamp # Move to inside if?
                if wait_for_updated_reading:
                    cur_time = init_time
                    while init_time == cur_time: # TODO: Use is_close?
                        lidar_data = self.stub.GetLatestLidarData(request_ack)
                        cur_time = lidar_data.timestamp
                
                angles = np.array(lidar_data.angles)
                dist = np.array(lidar_data.dists)
                print("Came at time: ", lidar_data.timestamp)
                lidar_data = np.stack((angles, dist), axis=1)

            elif self.connection == 'physical':
                init_time = float(self.redis_client.get('time')) # Move to inside if?
                lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)
                
                if wait_for_updated_reading:
                    cur_time = init_time
                    while init_time == cur_time: # TODO: Use is_close?
                        cur_time = float(self.redis_client.get('time'))
                        print("Waiting for updated lidar")
                    lidar_data = np.frombuffer(self.redis_client.get("lidar_data")).reshape(-1, 2)

                angles = lidar_data[:, 0]
                dist = lidar_data[:, 1]
            
            # Filter Noisy Lidar Points Close to Robot
            threshold = 90 # Units mm
            good_lidar_points_mask = lidar_data[:, 1] > threshold
            lidar_data = lidar_data[good_lidar_points_mask]

            # Fix Raw Lidar Output Format
            lidar_data[:, 0] = 360 - lidar_data[:, 0] # Make angles direction CCW
            lidar_data[:, 0] = np.deg2rad(lidar_data[:, 0]) # Convert Degrees to Radians

            angles = lidar_data[:, 0]
            dist = lidar_data[:, 1]

            cos = np.cos(angles)
            sin = np.sin(angles)

            x_coords = dist * cos
            y_coords = dist * sin
            z_coords = np.ones_like(x_coords)

            coords = np.stack((x_coords, y_coords, z_coords), axis=1)
            return coords, lidar_data, init_time

    def read_lidar_updated(self, manual_verification=False, wait_for_updated_reading=False):
        # coords, lidar_data = self._get_single_lidar_reading(wait_for_updated_reading)
        # if manual_verification:
        #     plt.scatter(coords[:, 0], coords[:, 1])
        #     plt.show()
        #     user_input = input("Do you want to reread the lidar?")
        #     while user_input == 'yes':
        #         coords, lidar_data = self._get_single_lidar_reading(wait_for_updated_reading)
        #         plt.scatter(coords[:, 0], coords[:, 1])
        #         plt.show()
        #         user_input = input("Do you want to reread the lidar?")
        # return coords, lidar_data

        user_input = 'yes'
        while user_input == 'yes':
            coords, lidar_data, _ = self._get_single_lidar_reading(wait_for_updated_reading)
            if manual_verification:
                plt.scatter(coords[:, 0], coords[:, 1])
                plt.show()
                user_input = input("Do you want to reread the lidar?")
            else:
                break
        return coords, lidar_data
    
    def read_rgb_camera(self):
        if self.connection == 'simulated':
            raise NotImplementedError
        elif self.connection == 'client':
            # Make RPC Call here 
            request_ack = pb2.Acknowledge(
                success=True,
                message="Client is ready for data!"
            )

            camera_data = self.stub.GetLatestImageData(request_ack)
            rgb_img = np.frombuffer(camera_data.rgb_img.img_bytes, dtype=np.uint8).reshape(480, 640, 3)
            depth_img = np.frombuffer(camera_data.depth_img.img_bytes, dtype=np.uint16).reshape(240, 320)
            return rgb_img, depth_img
        else:
            raise NotImplementedError

    def read_depth_camera(self):
        pass
    def read_imu(self):
        if self.connection == 'simulated':
            raise NotImplementedError
        elif self.connection == 'client':

            request_ack = pb2.Acknowledge(
                success=True,
                message="Client is ready for data!"
            )
            imu_data = self.stub.GetLatestIMUData(request_ack, timeout=2)

            accel_data = [imu_data.accel_x, imu_data.accel_y, imu_data.accel_z]
            gyro_data = [imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z]

            return accel_data, gyro_data
            
        elif self.connection == 'physical':
            raise NotImplementedError
    ### ------- Reading Sensors ------- ###
    
    def motion_command_to_pseudo_motor_diffs(self, motion_command):
        motion_type, pseudo_avg_motion = motion_command

        # Pseudo Motor Diff Examples:
        # [100.0, -100.0], # Forward 100.0 mm relative to heading 
        # [-np.pi/2, -np.pi/2], # Turn in place pi/2 radians CW TODO: CHECK DIRECTION

        if motion_type == 'linear':
            m = np.array([pseudo_avg_motion, -pseudo_avg_motion])
        elif motion_type == 'angular':
            m = np.array([pseudo_avg_motion, pseudo_avg_motion]) # TODO: Check if these need to be negated
        else:
            print("NEED TO RAISE BETTER EXCEPTION")
            raise NotImplementedError
        return m
    
    def predict_state(self, state, motor_position_differential):
        signs = np.sign(motor_position_differential)

        """
        Don't need to break down like this?

        Return: What should we return??
        """

        
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
            predicted_state = state + dx_state

        elif motion_type == 'angular':
            dx_state = np.array([0.0, 0.0, avg_motion])
            predicted_state = state + dx_state
        else:
            raise NotImplementedError

        return predicted_state

    def command_motion_and_predict_state(self, state, motion_command):
        m = self.command_motion_trial(motion_command)
        predicted_state = self.predict_state(state, m)
        return m, predicted_state
    
    def request_motion_command_from_user(self):
        """
        Returns: [status, motion_type, angular]
        """
        while True:
            command = input("Enter Robot Motion Command or type \"quit\":\n")

            if command == "quit":
                return ['', 0.0]

            try:
                # TODO: More useful exception messages here
                motion_type, dist = command.split(",")
                dist = float(dist)
                assert(motion_type == 'linear' or motion_type == 'angular')

                return [motion_type, dist]
            except:
                print("That was not a valid command, please try again!")
        
    def get_relative_transformation(self, motor_differentials):
        pass

    ### --- Motion Monitoring --- ###
    def local_planner(self, motion_command):
        motion_type, dist = motion_command
        if motion_type == 'linear':
            coords, raw_lidar_data = self.read_lidar_updated(wait_for_updated_reading=True) # Coords: (M, 3), Raw Lidar Data: (M, 2)

            cur_state = np.array([0.0, 0.0, 0.0]) # Current State (Local Frame)
            desired_state = np.array([dist, 0.0, 0.0]) # Desired State (Local Frame)

            # Filter Out Coords Near Robot
            p2p_dists = point_to_points_distance(cur_state[:2], coords[:, :2]).flatten()
            print(p2p_dists.shape)
            coords = coords[p2p_dists > self.local_planner_distance_threshold]

            line_segment = np.array([[0.0, 0.0, dist, 0.0]])
            dists = point_segment_distance(line_segment, coords[:, :2]) # (1, M)
            dists = dists.flatten()

            collision_coords_mask = dists < self.local_planner_distance_threshold

            if np.sum(collision_coords_mask) > 0:
                collision_coords = coords[collision_coords_mask]

                coord_dist_to_start = np.linalg.norm(collision_coords, axis=1)
                problem_coord = collision_coords[np.argmin(coord_dist_to_start)]

                problem_coord = problem_coord[:2]

                a = 1
                b = -2 * problem_coord[0]
                c = (problem_coord[0]**2 + problem_coord[1]**2 - self.local_planner_distance_threshold**2)
                
                sol1 = (-b + np.sqrt(b**2 - 4*a*c)) / 2
                sol2 = (-b - np.sqrt(b**2 - 4*a*c)) / 2

                dist_candidates = np.array([sol1, sol2])
                dist_idx = np.argmin(np.abs(dist_candidates))

                return ['linear', dist_candidates[dist_idx]]
                
        return motion_command
    
    def active_lidar_monitoring(self, pause_motion, thread_stop):

        while not thread_stop.is_set():
            _, raw_lidar_data = self.read_lidar_updated(wait_for_updated_reading=False)
            dists = raw_lidar_data[:, 1]
            min_dist = np.min(dists)

            if min_dist < self.active_distance_threshold:
                pause_motion.set()
            else:
                pause_motion.clear()

    def advanced_active_lidar_monitoring(self, pause_motion, thread_stop, desired_motor_pos_1, desired_motor_pos_2):
        
        while not thread_stop.is_set():
            current_motor_pos_1, current_motor_pos_2 = self.ri.get_motor_positions()

            pulse_diff_motor_1 = desired_motor_pos_1 - current_motor_pos_1
            revs_till_goal = pulse_diff_motor_1 / self.ri.pulse_per_rev
            dist_to_goal = revs_till_goal * self.wheel_circumference

            line_segment = np.array([[0.0, 0.0, dist_to_goal, 0.0]])
            coords, raw_lidar_data = self.read_lidar_updated(wait_for_updated_reading=False)

            # TODO: Filter Out Starting Points

            # Filter Out Coords Near Robot
            p2p_dists = point_to_points_distance(np.array([0.0, 0.0]), coords[:, :2]).flatten()
            coords = coords[p2p_dists > self.active_distance_threshold]

            dists = point_segment_distance(line_segment, coords[:, :2]) # (1, M)
            dists = dists.flatten()

            collision_coords_mask = dists < self.active_distance_threshold

            if np.sum(collision_coords_mask) > 0:
                pause_motion.set()
            else:
                pause_motion.clear()

    ## --- FOR TESTING ONLY -- ##
    def advanced_active_lidar_monitoring_with_visualization(self, pause_motion, thread_stop, desired_motor_pos_1, desired_motor_pos_2):

        rr.init(f"Visualize Lidar Brakes_{desired_motor_pos_2}", spawn=True)
        # start_time = time.time()
        while not thread_stop.is_set():
            current_motor_pos_1, current_motor_pos_2 = self.ri.get_motor_positions()

            pulse_diff_motor_1 = desired_motor_pos_1 - current_motor_pos_1
            revs_till_goal = pulse_diff_motor_1 / self.ri.pulse_per_rev
            dist_to_goal = revs_till_goal * self.wheel_circumference

            line_segment = np.array([[0.0, 0.0, dist_to_goal, 0.0]])
            coords, raw_lidar_data = self.read_lidar_updated(wait_for_updated_reading=False)

            # Filter Out Coords Near Robot
            p2p_dists = point_to_points_distance(np.array([0.0, 0.0]), coords[:, :2]).flatten()
            coords = coords[p2p_dists > self.active_distance_threshold]

            dists = point_segment_distance(line_segment, coords[:, :2]) # (1, M)
            dists = dists.flatten()

            collision_coords_mask = dists < self.active_distance_threshold

            if np.sum(collision_coords_mask) > 0:
                pause_motion.set()
            else:
                pause_motion.clear()

            # rr.set_time("time", duration=time.time()-start_time)
            rr.set_time("time", duration=time.time())
            rr.log("lidar points", rr.Points3D(coords))
            rr.log("robot location", rr.Points2D([[0.0, 0.0]], color=[0, 255, 0], radii=(self.wheelbase_length/2)))
        # rr.log_clear()
        rr.disconnect()
    ## --- FOR TESTING ONLY -- ##

    def is_moving(self):
        return np.sum(self.ri.get_motor_velocity()) > 0


    ### --- Motion Monitoring --- ###
    
        # TODO:
    # move_dist -> move_linear (DONE)
    # rotate_rad -> move_angular (DONE)
    # self.r -> move to Robot as self.wheel_radius (DONE)
    # self.L -> move to Robot as self.wheelbase_length (DONE)
    # Swap Prints to Logs

    ### --- Commanding Robot Motions --- ###
    
    def move_linear(self, mm=100):
        logger.info(f"Moving Linear: {mm} mm")
        init_motor_pos = self.ri.get_motor_positions()
        # print(f"Initial Motor Positions: {init_motor_pos}")
        logger.info(f"Initial Motor Positions: {init_motor_pos}")

        cir = self.r * 2 * np.pi

        required_revs = mm / cir

        req_pulse = int(required_revs * self.ri.pulse_per_rev)

        desired_motor_pos_1 = init_motor_pos[0] + req_pulse
        desired_motor_pos_2 = init_motor_pos[1] - req_pulse
        logger.info(f"Desired Motor Positions: {[desired_motor_pos_1, desired_motor_pos_2]}")


        self.controller.set_position(id=1, position=desired_motor_pos_1)
        self.controller.set_position(id=2, position=desired_motor_pos_2)

        if self.guard_active_motion:
            thread_stop = threading.Event() # Thread Shared Variable
            pause_motion = threading.Event() # Thread Shared Variable
            was_paused = False # Function Local Variable

            # Start the Monitoring Thread
            # monitoring_thread = threading.Thread(target=self.active_lidar_monitoring, args=(pause_motion, thread_stop,))
            monitoring_thread = threading.Thread(target=self.advanced_active_lidar_monitoring, args=(pause_motion, thread_stop, desired_motor_pos_1, desired_motor_pos_2,))
            # monitoring_thread = threading.Thread(target=self.advanced_active_lidar_monitoring_with_visualization, args=(pause_motion, thread_stop, desired_motor_pos_1, desired_motor_pos_2,))
            monitoring_thread.start()

            while np.sum(self.ri.get_motor_velocity()) > 0:
                while pause_motion.is_set(): # Check this condition??
                    # print("Disabling Motion")
                    self.ri.set_torque(TORQUE_DISABLE)

                self.ri.set_torque(TORQUE_ENABLE)
                # print("Continuing Movement")
                self.controller.set_position(id=1, position=desired_motor_pos_1)
                self.controller.set_position(id=2, position=desired_motor_pos_2)

            # TODO: Try out this method of pausing the motors
            # while np.sum(self.ri.get_motor_velocity()) > 0:
            #     while pause_motion.is_set(): # Check this condition??
            #         # print("Disabling Motion")
            #         current_motor_pos_1, current_motor_pos_2 = self.ri.get_motor_positions()
            #         self.controller.set_position(id=1, position=current_motor_pos_1)
            #         self.controller.set_position(id=2, position=current_motor_pos_2)

            #     # print("Continuing Movement")
            #     self.controller.set_position(id=1, position=desired_motor_pos_1)
            #     self.controller.set_position(id=2, position=desired_motor_pos_2)

            # REFERNCE: Working Pause Mechanism on the Robot
            # while self.is_moving():
            #     while pause_motion.is_set(): # Check this condition??
            #         if not was_paused:
            #             print("Disabling Motion")
            #             self.controller_lock.acquire()
            #             self.ri.set_torque(TORQUE_DISABLE)
            #             self.controller_lock.release()
            #             was_paused = True

            #     if was_paused:
            #         self.controller_lock.acquire()
            #         self.ri.set_torque(TORQUE_ENABLE)
            #         self.controller_lock.release()
            #         print("Continuing Movement")
            #         self.controller_lock.acquire()
            #         self.controller.set_position(id=1, position=desired_motor_pos_1)
            #         self.controller.set_position(id=2, position=desired_motor_pos_2)
            #         self.controller_lock.release()
            #         was_paused = False

            # Stop the Monitoring Thread
            thread_stop.set()
            monitoring_thread.join()
        else:

            while np.sum(self.ri.get_motor_velocity()) > 0:
                continue
        
        final_motor_pos = self.ri.get_motor_positions()


        if desired_motor_pos_1 < 0:
            logger.info("Deflating final motor position 1")
            final_motor_pos[0] -= self.controller.max_motor_position

        if desired_motor_pos_2 < 0:
            logger.info("Deflating final motor position 2")
            final_motor_pos[1] -= self.controller.max_motor_position

        logger.info(f"Final Motor Positions: {final_motor_pos}")
        return self.compute_linear_motion(init_motor_pos, final_motor_pos)
    
    def move_angular(self, rad=np.pi/2):
        logger.info(f"Moving Angular: {rad} radians")
        init_motor_pos = self.ri.get_motor_positions()
        logger.info(f"Initial Motor Positions: {init_motor_pos}")

        cir = self.r * 2 * np.pi
        body_cir = 2 * np.pi * (self.L / 2)

        rotation_percentage = -rad / (2 * np.pi)

        wheel_travel_dist = body_cir * rotation_percentage

        required_revs = wheel_travel_dist / cir

        req_pulse = int(required_revs * self.ri.pulse_per_rev)

        desired_motor_pos_1 = init_motor_pos[0] + req_pulse
        desired_motor_pos_2 = init_motor_pos[1] + req_pulse
        logger.info(f"Desired Motor Positions: {[desired_motor_pos_1, desired_motor_pos_2]}")

        # TODO: Is this bad design??
        self.controller.set_position(id=1, position=desired_motor_pos_1)
        self.controller.set_position(id=2, position=desired_motor_pos_2)

        while np.sum(self.ri.get_motor_velocity()) > 0:
            continue
        
        final_motor_pos = self.ri.get_motor_positions()
        if desired_motor_pos_1 < 0:
            logger.info("Deflating final motor position 1")
            final_motor_pos[0] -= self.controller.max_motor_position
            
        if desired_motor_pos_2 < 0:
            logger.info("Deflating final motor position 2")
            final_motor_pos[1] -= self.controller.max_motor_position
        
        logger.info(f"Final Motor Positions: {final_motor_pos}")
        return self.compute_rotation_motion(init_motor_pos, final_motor_pos)
    

    def compute_linear_motion(self, init_mp, final_mp):
        logger.debug(f"Compute Linear Motion")
        init_mp = np.array(init_mp)
        final_mp = np.array(final_mp)
        logger.debug(f"Initial Motor Positions: {init_mp}")
        logger.debug(f"Final Motor Positions: {init_mp}")

        diff = final_mp - init_mp
        logger.debug(f"Motor Differentials: {diff}")

        revs = diff / self.ri.pulse_per_rev

        cir = np.pi * 66.5 # TODO: Avoid Magic Number

        dists = revs * cir 
        return dists
    
    def compute_rotation_motion(self, init_mp, final_mp):
        logger.debug(f"Compute Rotation Motion")
        init_mp = np.array(init_mp)
        final_mp = np.array(final_mp)
        logger.debug(f"Initial Motor Positions: {init_mp}")
        logger.debug(f"Final Motor Positions: {init_mp}")

        motor_pos_diff = final_mp - init_mp
        logger.debug(f"Motor Differentials: {motor_pos_diff}")

        rev_diff = motor_pos_diff / self.ri.pulse_per_rev
        # rot_diff = ri.r * rev_diff / (ri.L / 2)
        rot_diff = 2 * self.r * rev_diff / (self.L)

        rad_rotated = rot_diff * (2*np.pi)

        return rad_rotated

    def command_motion_trial(self, motion_command):
        """
        Return motion differential
        """
        print(f"Original Motion Command: {motion_command}")
        # motion_command = self.local_planner(motion_command)
        print(f"Locally Planned Motion Command: {motion_command}")
        motion_type, dist = motion_command
        if self.connection == 'simulated':
            m = self.motion_command_to_pseudo_motor_diffs(motion_command)
        elif self.connection == 'client':
            # Make RPC Call to robot (For now just return the same m...)
            rpc_motion_command = pb2.MotionCommand(
                motion_type=motion_command[0],
                distance=motion_command[1]
            )
            motion_distance = self.stub.SendMotionCommand(rpc_motion_command)
            m = np.array([motion_distance.left_wheel_dist, motion_distance.right_wheel_dist])

        elif self.connection == 'physical':
            # Consider moving this to RobotInterface in a function called "ExecuteMotion or CommandMotion or..."
            if motion_type == 'linear':
                # m = self.ri.move_dist(dist) # TODO: Remove after successful rollout
                m = self.move_linear(dist)
            elif motion_type == 'angular':
                # m = self.ri.rotate_rad(dist) # TODO: Remove after successful rollout
                m = self.move_angular(dist)
            else:
                raise NotImplementedError
        return m
    
    ### --- Commanding Robot Motions --- ###

    
    # TODO: Add ability to toggle between grid coords and world coords
    def draw_state(self, ax, state):
        x, y, theta = state

        # Draw Robot Body
        relative_robot_radius = 100 # TODO: Compute this value
        robot_outline = Point([x, y]).buffer(relative_robot_radius)
        ax.fill(*robot_outline.exterior.xy, color='blue')

        # Draw Wheels
        wheel_width = 50
        wheel_height = 20

        wheel_offset = relative_robot_radius + wheel_height/2 # TODO: Compute this value
        left_wheel_y_loc = y + wheel_offset
        right_wheel_y_loc = y - wheel_offset

        

        left_wheel = create_rectangle_geometry(x, left_wheel_y_loc, wheel_width, wheel_height)
        right_wheel = create_rectangle_geometry(x, right_wheel_y_loc, wheel_width, wheel_height)

        left_wheel = affinity.rotate(left_wheel, theta, use_radians=True, origin=[x, y])
        right_wheel = affinity.rotate(right_wheel, theta, use_radians=True, origin=[x, y])
        ax.fill(*left_wheel.exterior.xy, color='black')
        ax.fill(*right_wheel.exterior.xy, color='black')

        # Draw Heading Line
        heading_line_length = 500
        heading_line_ep = np.array([np.cos(theta), np.sin(theta)]) * heading_line_length + np.array([x, y])
        ax.plot([x, heading_line_ep[0]], [y, heading_line_ep[1]], color='red')

    # TODO: Optimize this function
    def state_pairs_to_motion_commands(self, state1, state2):
        state_transition_motion_commands = []

        x1, y1, theta1 = state1
        x2, y2, theta2 = state2

        positional_difference_vector = np.arctan2(y2-y1, x2-x1)

        initial_turn = positional_difference_vector - theta1
        linear_motion = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        final_turn = theta2 - positional_difference_vector

        # Normalize Angles:
        initial_turn = np.arctan2(np.sin(initial_turn), np.cos(initial_turn))
        final_turn = np.arctan2(np.sin(final_turn), np.cos(final_turn))

        # Add the motion commands in order (Skip those that are 0 [unlikely to happen...])
        if initial_turn != 0:
            state_transition_motion_commands.append(["angular", initial_turn])
        if linear_motion != 0:
            state_transition_motion_commands.append(["linear", linear_motion])
        if final_turn != 0:
            state_transition_motion_commands.append(["angular", final_turn])
        
        return state_transition_motion_commands

    def path_to_motion_commands(self, path):
        motion_commands = []
        for i in range(len(path)-1):
            state1 = path[i]
            state2 = path[i+1]

            state_transition_motion_commands = self.state_pairs_to_motion_commands(state1, state2)
            motion_commands.extend(state_transition_motion_commands)
        return motion_commands
    
    def terminate(self):
        pass
    
if __name__ == "__main__":

    robot = Robot(connection='simulated')
    # robot.command_motion_trial(['linear', -400])
    # robot.command_motion_trial(['angular', np.pi/2])
    robot.command_motion_trial(['linear', 400])

    # motions = [
    #     ('linear', 400),
    #     ('angular', -np.pi/2),
    #     ('linear', 700),
    # ]
    # for motion_type, dist in motions:
    #     robot.command_motion_trial([motion_type, dist])
    # coords = robot.read_lidar_trial()

    # print(coords)





    

    
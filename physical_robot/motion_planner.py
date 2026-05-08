import numpy as np
import pickle
import matplotlib.pyplot as plt

from physical_robot.robot import Robot, PhysicalRobotSpace
from physical_robot.maps import Map, SemanticMap
from physical_robot.robot.robot_utils import localize_robot, mpc_plan_and_follow_trajectory
from motion_planning.search import RRT, PRM
from physical_robot.models.vlm.vlm_client import VLMClient
from physical_robot.algorithms.icp import run_icp
from physical_robot.utils import transformation_mat_to_state, register_logger

# register_logger(__name__, 'path_tracker', )

class PathTracker():
    def __init__(self, robot_space: PhysicalRobotSpace, motion_planner: RRT | PRM = None):
        self.robot_space = robot_space
        self.map = self.robot_space.map

        self.path_lookahead_distance = 4

        self.motion_planner = motion_planner
    
    def localize_robot(self):
        self.robot_location, self.pf = localize_robot(self.robot_space, self.map)

    def is_close(self, state1, state2, threshold=100):
        return np.linalg.norm(state1[:2] - state2[:2]) < threshold
    
    def track_full_path(self, path: list[np.ndarray], do_replan=False):
        current_state = self.robot_location
        target = path[-1]
        
        # Standardize lookahead: replanning usually needs a specific distance, 
        # while one-shot execution often uses the full path (-1).
        lookahead = self.path_lookahead_distance if do_replan else -1

        while True:
            # 1. Convert path to motion commands
            # Ensuring we consistently pass the expected format to the robot_space
            motion_commands = self.robot_space.path_to_motion_commands(path)

            # 2. Execute the motion
            current_state = self.step_motion_executions(
                current_state, 
                motion_commands, 
                lookahead_dist=lookahead
            )

            # 3. Check exit conditions
            # If we aren't replanning, or if we've reached the goal, we stop.
            if not do_replan or self.is_close(current_state.value, target.value):
                break

            # 4. Replan for the next iteration
            try:
                new_path_objects = self.motion_planner.search(current_state.value, target.value)
                path = [p.value for p in new_path_objects]
            except Exception as e: # TODO: Make sure this only catches the motion_planner.search being None exception
                raise ValueError("Need to pass in a motion planner to use the do_replan feature")


    def step_single_motion_exection(self, current_state, motion_command, min_motion_threshold=0.09):
        _, motion_dist = motion_command
        if abs(motion_dist) < min_motion_threshold:
            print(f"Skipping Motion: {motion_command}")
            return current_state
        
        print(f"Executing Motion Command: {motion_command}")
        m, predicted_state = self.robot_space.command_motion_and_predict_state(current_state.value, motion_command)
        coords, lidar_data = self.robot_space.read_lidar_updated(wait_for_updated_reading=True, manual_verification=False)

        # Particle Filter State Prediction Update
        current_state = self.pf.step(motion_delta=motion_command, scan=lidar_data)
        print(f"Current State After Particle Filter Update: {np.round(current_state, 2)}")

        # ICP State Refinement
        T = run_icp(coords, self.map.get_points(), current_state, filter_init_outliers=False, visualize=False)
        current_state = self.robot_space.make_state(transformation_mat_to_state(T))
        print(f"Current State After ICP Refinement: {np.round(current_state.value, 2)}")
        return current_state

    def step_motion_executions(self, current_state, motion_commands, lookahead_dist=-1):
        for i, motion_command in enumerate(motion_commands):
            current_state = self.step_single_motion_exection(current_state, motion_command)

            # Break (to likely replan) if we are on the lookahead_dist-th Motion Command
            if lookahead_dist == i:
                break

        return current_state
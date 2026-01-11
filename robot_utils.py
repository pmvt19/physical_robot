import numpy as np
import matplotlib.pyplot as plt

from particle_filter import ParticleFilter
from robot import Robot
from robot_space import PhysicalRobotSpace
from map import Map
from icp import run_icp
from utils import transformation_mat_to_state

from motion_planning.prm import PRM
from motion_planning.state import NumpyState

def is_close(state1, state2, threshold=100):
    return np.linalg.norm(state1[:2] - state2[:2]) < threshold

def localize_robot(robot: Robot, 
                   map: Map, 
                   num_particles: int = 10000, 
                   localizing_motion: str = 'in-place', 
                   custom_localizing_motion: list[tuple] | None = None, 
                   visualize: bool = True) -> tuple[np.ndarray, ParticleFilter]:

    pf = ParticleFilter(map_obj=map)
    pf.initialize(num_particles=num_particles)

    if localizing_motion == 'in-place':
        motion_commands = [['angular', np.pi/2],
                           ['angular', np.pi/2],
                           ['angular', np.pi/2],
                           ['angular', np.pi/2]]
    elif localizing_motion == 'single in-place':
        motion_commands = [['angular', np.pi/2]]
    elif localizing_motion == 'square':
        linear_dist = 10.0
        motion_commands = [['linear', linear_dist],
                           ['angular', np.pi/2],
                           ['linear', linear_dist],
                           ['angular', np.pi/2],
                           ['linear', linear_dist],
                           ['angular', np.pi/2],
                           ['linear', linear_dist],
                           ['angular', np.pi/2],]
    elif localizing_motion == 'custom':
        if custom_localizing_motion:
            motion_commands = custom_localizing_motion
        else:
            raise ValueError('"custom" set for localizing motion but no custom motions was given via the custom_localizing_motion argument')

    for motion_command in motion_commands:
        m = robot.command_motion_trial(motion_command)
        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)

    if visualize:
        pf.visualize_particles(plt.gca())
        pf.map.visualize_points(plt.gca())
        plt.scatter(updated_state[0], updated_state[1], color='orange', zorder=2)
        plt.show()
    return updated_state, pf

def mpc_plan_and_follow_trajectory(robot: PhysicalRobotSpace,
                                   pf: ParticleFilter,
                                   map: Map,
                                   prm: PRM,
                                   start: NumpyState,
                                   target: NumpyState):
    
    path = prm.search(start, target)
    path = [p.value for p in path]
    current_state = start

    while not is_close(current_state.value, target.value):
        motion_commands = robot.path_to_motion_commands(path)
        
        for i, motion_command in enumerate(motion_commands):
            print(f"Executing Motion Command: {motion_command}")
            m, predicted_state = robot.command_motion_and_predict_state(current_state.value, motion_command)
            
            coords, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True, manual_verification=True)

            # Particle Filter State Prediction Update
            current_state = pf.step(motion_delta=motion_command, scan=lidar_data)
            print(f"Current State After Particle Filter Update: {current_state}")

            # ICP State Refinement
            T = run_icp(coords, map.get_points(), current_state, filter_init_outliers=False, visualize=True)
            current_state = robot.make_state(transformation_mat_to_state(T))
            print(f"Current State After Refinement: {current_state}")

            # Break if we are on the 3rd Motion Command
            if i == 2:
                break
        
        path = prm.search(current_state, target)
        path = [p.value for p in path]
        motion_commands = robot.path_to_motion_commands(path)
            
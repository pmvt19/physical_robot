import numpy as np
import matplotlib.pyplot as plt
import pickle
import os

from icp import run_icp
from robot import Robot
from map import Map
from utils import transformation_mat_to_state
from test_utils import generate_fake_map, load_saved_map, load_saved_advanced_map
from motion_planning.space import RobotSpace
from motion_planning.state import NumpyState, AngularNumpyState
from motion_planning.utils import numpystate_distance, smooth_path

from motion_planning.rrt import RRT
from motion_planning.prm import PRM

from robot_space import PhysicalRobotSpace
from particle_filter import ParticleFilter

from robot_utils import localize_robot, mpc_plan_and_follow_trajectory

def is_close(state1, state2, threshold=100):
    return np.linalg.norm(state1[:2] - state2[:2]) < threshold

def run_motion_planning():
    # Load Advanced Map
    mymap = load_saved_advanced_map(directory='saves/scenes/extensive_apartment')
    
    # Initialize Robot Space
    robot = PhysicalRobotSpace(mymap)

    # Localize Robot within Map
    start, pf = localize_robot(robot, robot.map)

    # Create and Build PRM
    prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    prm.create_graph()

    # TODO: REMOVE THE HARDCODED TARGET
    target = robot.make_state(np.array([-1424.0, 656.0, 0.0])) # Left Side of Desk
    mpc_plan_and_follow_trajectory(robot, pf, robot.map, prm, robot.make_state(start), target)

if __name__ == '__main__':
    seed = np.random.randint(10000)
    print(f"Using Seed: {seed}")
    np.random.seed(seed)

    run_motion_planning()

    # target = robot.make_state(np.array([-1664.0, -1584.0, 0.0])) # Inside Bedroom
    # target = robot.make_state(np.array([2016.0, -834.0, 0.0])) # In Front of Bathroom
    # target = robot.make_state(np.array([-1424.0, 656.0, 0.0])) # Left Side of Desk
    # target = robot.make_state(np.array([190.0, 2229.0, 0.0])) # Next to Desk
    # localize_mpc_planning(robot, target)

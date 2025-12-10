import numpy as np
import matplotlib.pyplot as plt
import pickle
import os

from icp import run_icp
from robot import Robot
from map import Map
from utils import transformation_mat_to_state
from test_utils import generate_fake_map, load_saved_map
from motion_planning.space import RobotSpace
from motion_planning.state import NumpyState, AngularNumpyState
from motion_planning.utils import numpystate_distance, smooth_path

from motion_planning.rrt import RRT
from motion_planning.prm import PRM

from robot_space import PhysicalRobotSpace
from particle_filter import ParticleFilter

def is_close(state1, state2, threshold=100):
    return np.linalg.norm(state1[:2] - state2[:2]) < threshold

def prm_plan(robot : PhysicalRobotSpace, prm : PRM, start : NumpyState, target : NumpyState):
    # start = robot.make_state(robot.state)
    path = prm.search(start, target)
    return path

def localize_robot(robot : PhysicalRobotSpace, pf : ParticleFilter):
    # motion_commands = [['angular', 1.57],
    #                    ['angular', 1.57],
    #                    ['angular', 1.57],
    #                    ['angular', 1.57],]
    motion_commands = [['angular', 1.57],]
    for motion_command in motion_commands:
        m = robot.command_motion_trial(motion_command)
        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)

    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    # plt.scatter(scan[:, 0], scan[:, 1], color='purple')
    plt.scatter(updated_state[0], updated_state[1], color='orange', zorder=2)
    plt.show()
    return updated_state

def create_or_load_prm(scene, robot) -> PRM:
    planning_dir = f'saves/scenes/{scene}/planning'
    # prm_path = f'saves/scenes/{scene}/planning/prm_graph.pickle'
    prm_path = f'{planning_dir}/prm_graph.pickle'

    # prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    prm = None
    # if os.path.exists(prm_path) and False:
    if os.path.exists(prm_path):
        prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=False)
        prm_graph = pickle.load(open(prm_path, 'rb'))
        prm.graph = prm_graph
    else:
        prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
        prm.create_graph()
        os.makedirs(planning_dir, exist_ok=True)
        pickle.dump(prm.graph, open(prm_path, 'wb'))
    return prm


def localize_mpc_planning(robot : PhysicalRobotSpace, target : NumpyState):
    pf = ParticleFilter(map_obj=robot.map)
    pf.initialize(num_particles=10000)

    current_state = localize_robot(robot, pf)

    prm : PRM = create_or_load_prm(scene='tmp')
    

    

    while not is_close(current_state, target.value):
        print("Replanning Path")
        path = prm_plan(robot, prm, robot.make_state(current_state), target)
        mymap.visualize_points(plt.gca())
        robot.draw_state(plt.gca(), current_state)
        robot.draw_state(plt.gca(), target.value)
        prm.draw(plt.gca())
        plt.show()
        robot.map.visualize_points(plt.gca())
        for p in path:
            robot.draw_state(plt.gca(), p.value)
        plt.show()
        path_segment = path[:4]
        path_segment = [p.value for p in path_segment]

        motion_commands = robot.path_to_motion_commands(path_segment)
        # motion_commands = robot.smooth_motion_commands(motion_commands)
        
        # current_state = start_state_value
        for motion_command in motion_commands:
            print(f"Executing Motion Command: {motion_command}")
            # m = robot.command_motion_trial(motion_command)
            m, predicted_state = robot.command_motion_and_predict_state(current_state, motion_command)

            coords, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

            updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)
            current_state = updated_state

            # Refine the State:
            T = run_icp(coords, mymap.get_points(), current_state, filter_init_outliers=False, visualize=False)
            refined_state = transformation_mat_to_state(T)
            current_state = refined_state
            print(f"Current State: {current_state}")

if __name__ == '__main__':
    seed = np.random.randint(10000)
    print(f"Using Seed: {seed}")
    np.random.seed(seed)

    mymap = load_saved_map(directory='saves/scenes/tmp/map')
    robot = PhysicalRobotSpace(mymap)
    robot.edge_validity_delta = 200.0

    mymap.visualize_points(plt.gca())
    plt.show()

    ## For tmp map

    # target = robot.make_state(np.array([0.0, 0.0, 0.0])) # Origin (In front of bathroom door)
    # target = robot.make_state(np.array([-3855.0, -691.0, 0.0])) # In front of bedroom door
    # target = robot.make_state(np.array([-1306.0, 2141.0, 0.0])) # Next to desk
    target = robot.make_state(np.array([1785.0, 1899.0, 0.0])) # In front of fridge
    localize_mpc_planning(robot, target)

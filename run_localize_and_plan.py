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
    motion_commands = [['angular', 1.57],
                       ['angular', 1.57],
                       ['angular', 1.57],
                       ['angular', 1.57],]
    for motion_command in motion_commands:
        m = robot.command_motion_trial(motion_command)
        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        # FIXED??
        # ## TODO: CLEAN THIS HACK
        # lidar_data = np.copy(lidar_data)
        # lidar_data[:, 0] = 360 - lidar_data[:, 0]
        # lidar_data[:, 0] = lidar_data[:, 0] + 90
        # lidar_data[:, 0] = lidar_data[:, 0] % 360
        # lidar_data[:, 0] = np.deg2rad(lidar_data[:, 0])
        # ## TODO: CLEAN THIS HACK

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)

    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    # plt.scatter(scan[:, 0], scan[:, 1], color='purple')
    plt.scatter(updated_state[0], updated_state[1], color='orange', zorder=2)
    plt.show()
    return updated_state

def create_or_load_prm(scene) -> PRM:
    planning_dir = f'saves/scenes/{scene}/planning'
    # prm_path = f'saves/scenes/{scene}/planning/prm_graph.pickle'
    prm_path = f'{planning_dir}/prm_graph.pickle'

    prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    if os.path.exists(prm_path):
        prm_graph = pickle.load(open('dumps/run1/prm_graph.pickle', 'rb'))
        prm.graph = prm_graph
    else:
        prm.create_graph()
        os.makedirs(planning_dir, exist_ok=True)
        pickle.dump(prm.graph, open(prm_path, 'wb'))


def localize_mpc_planning(robot : PhysicalRobotSpace, target : NumpyState):
    pf = ParticleFilter(map_obj=robot.map)
    pf.initialize(num_particles=10000)

    current_state = localize_robot(robot, pf)
    
    # pickle.dump(open('dumps/run1/prm.pickle', 'wb'))
    
    # prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    # prm.create_graph()
    # pickle.dump(prm.graph, open('dumps/run1/prm_graph.pickle', 'wb'))
    
    # prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    # prm_graph = pickle.load(open('dumps/run1/prm_graph.pickle', 'rb'))
    # prm.graph = prm_graph

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
        motion_commands = robot.smooth_motion_commands(motion_commands)
        
        # current_state = start_state_value
        for motion_command in motion_commands:
            print(f"Executing Motion Command: {motion_command}")
            # m = robot.command_motion_trial(motion_command)
            m, predicted_state = robot.command_motion_and_predict_state(current_state, motion_command)

            scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

            # # FIXED??
            # ## TODO: CLEAN THIS HACK
            # lidar_data = np.copy(lidar_data)
            # lidar_data[:, 0] = 360 - lidar_data[:, 0]
            # lidar_data[:, 0] = lidar_data[:, 0] + 90
            # lidar_data[:, 0] = lidar_data[:, 0] % 360
            # lidar_data[:, 0] = np.deg2rad(lidar_data[:, 0])
            # ## TODO: CLEAN THIS HACK

            updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)
            current_state = updated_state

            # Refine the State:
            T = run_icp(scan, mymap.get_points(), current_state, filter_init_outliers=False, visualize=False)
            refined_state = transformation_mat_to_state(T)
            current_state = refined_state
            print(f"Current State: {current_state}")
        



def localize_robot(robot : PhysicalRobotSpace):
    pf = ParticleFilter(map_obj=robot.map)
    pf.initialize(num_particles=10000)

    motion_commands = [['angular', 1.57],
                       ['angular', 1.57],
                       ['angular', 1.57],
                       ['angular', 1.57],]
    for motion_command in motion_commands:
        m = robot.command_motion_trial(motion_command)
        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        ## TODO: CLEAN THIS HACK
        lidar_data = np.copy(lidar_data)
        lidar_data[:, 0] = 360 - lidar_data[:, 0]
        lidar_data[:, 0] = lidar_data[:, 0] + 90
        lidar_data[:, 0] = lidar_data[:, 0] % 360
        lidar_data[:, 0] = np.deg2rad(lidar_data[:, 0])
        ## TODO: CLEAN THIS HACK

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)

    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    # plt.scatter(scan[:, 0], scan[:, 1], color='purple')
    plt.scatter(updated_state[0], updated_state[1], color='orange', zorder=2)
    plt.show()
    return updated_state

# def localize_and_move(robot : PhysicalRobotSpace, target):
#     pass

if __name__ == '__main__':
    seed = np.random.randint(10000)
    print(f"Using Seed: {seed}")
    np.random.seed(seed)

    mymap = load_saved_map(directory='dumps/run1')
    robot = PhysicalRobotSpace(mymap)
    robot.edge_validity_delta = 200.0

    mymap.visualize_points(plt.gca())
    plt.show()

    # target = robot.make_state(np.array([1291.0, -1255.0, 0.0])) # In front of bathroom door
    # target = robot.make_state(np.array([-1148.0, -4425.0, 0.0])) # Inside of bedroom
    target = robot.make_state(np.array([3039.0, 1715.0, 0.0])) # In front of fridge

    localize_mpc_planning(robot, target)



    exit()
    np.set_printoptions(precision=3, suppress=True)
    seed = np.random.randint(10000)
    print(f"Using Seed: {seed}")
    np.random.seed(seed)

    # Example usage
    mymap = load_saved_map(directory='dumps/run1')
    # mymap.inflate_obstacles(kernel_size=3)
    mymap.visualize_points(plt.gca())
    plt.show()

    robot = PhysicalRobotSpace(mymap)
    # robot.edge_validity_delta = 0.1
    robot.edge_validity_delta = 200
    # rrt = RRT(env=robot)
    prm = PRM(env=robot, num_samples=10000, num_neighbors=10)
    prm.create_graph()
    # rrt.delta = 100.0



    # start = robot.make_state(np.array([1286.0, -1288.0, 0.0]))
    start_state_value = localize_robot(robot)
    start = robot.make_state(start_state_value)
    print(start.value)
    # exit()
    # target = robot.make_state(np.array([-2485.0, 113.0, 0.0]))
    # target = robot.make_state(np.array([1291.0, -1255.0, 0.0])) # In front of bathroom door
    # target = robot.make_state(np.array([-2687.0, -1618.0, 0.0])) # In Middle of bedroom door
    target = robot.make_state(np.array([-1148.0, -4425.0, 0.0])) # Inside of bedroom
    # path = rrt.search(start=start, target=target, max_steps=10000)
    path = prm.search(start=start, target=target)


    mymap.visualize_points(plt.gca())
    robot.draw_state(plt.gca(), start.value)
    robot.draw_state(plt.gca(), target.value)
    # rrt.draw_tree(plt.gca())
    prm.draw(plt.gca())
    plt.show()
    # print(f"RRT tree size: {len(rrt.tree)}")

    print(f"Found path with {len(path)} states.")
    mymap.visualize_points(plt.gca())
    # prm.draw_graph(plt.gca())
    for p in path:
        robot.draw_state(plt.gca(), p.value)
    plt.show()

    # for p in path:
    #     plt.cla()
    #     mymap.visualize_points(plt.gca())
    #     robot.draw_state(plt.gca(), p.value)
    #     plt.pause(0.1)

    # smoothed_path = smooth_path(robot, path)
    smoothed_path = path
    path = [state.value for state in path]
    smoothed_path = [state.value for state in smoothed_path]
    np.set_printoptions(precision=2, suppress=True)
    for state in path:
        print(state)
    for state in smoothed_path:
        print(state)

    mymap.visualize_points(plt.gca())
    for p in smoothed_path:
        robot.draw_state(plt.gca(), p)
    plt.show()

    pf = ParticleFilter(map_obj=robot.map)
    pf.initialize(num_particles=10000)
    

    motion_commands = robot.path_to_motion_commands(smoothed_path)
    print(motion_commands)
    motion_commands = robot.smooth_motion_commands(motion_commands)
    print(motion_commands)
    current_state = start_state_value
    for motion_command in motion_commands:
        print(f"Executing Motion Command: {motion_command}")
        # m = robot.command_motion_trial(motion_command)
        m, predicted_state = robot.command_motion_and_predict_state(current_state, motion_command)

        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        ## TODO: CLEAN THIS HACK
        lidar_data = np.copy(lidar_data)
        lidar_data[:, 0] = 360 - lidar_data[:, 0]
        lidar_data[:, 0] = lidar_data[:, 0] + 90
        lidar_data[:, 0] = lidar_data[:, 0] % 360
        lidar_data[:, 0] = np.deg2rad(lidar_data[:, 0])
        ## TODO: CLEAN THIS HACK

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)
        print(f"Predicted State: {predicted_state}")
        print(f"Updated State: {updated_state}")

        current_state = updated_state
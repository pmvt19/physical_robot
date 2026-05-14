import matplotlib.pyplot as plt
import numpy as np

from physical_robot.robot import Robot
from physical_robot.robot.robot_space import PhysicalRobotSpace
from physical_robot.task_planner.task_planner import TaskPlanner
from physical_robot.utils.test_utils import load_saved_semantic_map, load_saved_advanced_map
from physical_robot.robot.robot_utils import localize_robot, path_obj_to_list_of_states
from physical_robot.motion_planner import PathTracker
from motion_planning.search import RRT

def run_navigation():
    # Initialize Objects
    robot = Robot(connection='client')
    mymap = load_saved_advanced_map('saves/scenes/refactor_test_map')

    task_planner = TaskPlanner(robot, mymap)
    robot_space = PhysicalRobotSpace(mymap)
    rrt = RRT(robot_space, delta=500.0)
    path_tracker = PathTracker(robot_space, rrt)

    # Get the Target Pose
    target = task_planner.get_target_pose()
    
    # Define Start
    path_tracker.localize_robot()
    start = robot_space.make_state(path_tracker.robot_location)
    
    path = rrt.search(start, target)
    path = path_obj_to_list_of_states(path)

    path_tracker.track_full_path(path, do_replan=True)

if __name__ == '__main__':
    run_navigation()
    



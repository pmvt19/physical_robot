import math
import numpy as np
import matplotlib.pyplot as plt

from robot import Robot
from map import Map
from test_utils import generate_fake_map, load_saved_map
from motion_planning.space import RobotSpace
from motion_planning.state import AngularNumpyState
from motion_planning.utils import numpystate_distance, smooth_path

class PhysicalRobotSpace(Robot, RobotSpace):
    def __init__(self, map_obj):
        Robot.__init__(self, connection='client')
        RobotSpace.__init__(self)

        self.map : Map = map_obj

        self.angular_dims_start = 2

        # Setting Edge Validity Delta to Acceptable Value (mm units)
        self.edge_validity_delta = 200.0

    def is_valid(self, state):
        raise NotImplementedError
        state = self.get_state_value(state)
        # Implement collision checking or other validity checks here
        points = self.map.get_points()
        map_circles = np.concatenate((points, np.ones((points.shape[0], 1), dtype=np.float32) * (self.map.resolution * 14)), axis=1)

        robot_circle = np.array([state[0], state[1], self.radius*2])

        dists = np.sqrt((map_circles[:, 0] - robot_circle[0])**2 + (map_circles[:, 1] - robot_circle[1])**2)
        if np.any(dists < (map_circles[:, 2] + robot_circle[2])):
            return False
        return True
    
    def circles_to_validity(self, obstacle_circles, robot_circles):
        dist_mat = np.sqrt(np.sum(robot_circles[:, :2]**2, axis=1, keepdims=True) + np.sum(obstacle_circles[:, :2]**2, axis=1, keepdims=True).T + (-2 * (robot_circles[:, :2] @ obstacle_circles[:, :2].T)))
        min_dists = robot_circles[:, 2].reshape(-1, 1) + obstacle_circles[:, 2].reshape(1, -1)
        validity_mask = dist_mat > min_dists
        validities = np.all(validity_mask, axis=1)
        return validities
    
    def batch_is_valid(self, states):
        # points = self.map.get_points()
        # # map_circles = np.concatenate((points, np.ones((points.shape[0], 1), dtype=np.float32) * (self.map.resolution * 14)), axis=1) # Kinda works 
        # map_circles = np.concatenate((points, np.ones((points.shape[0], 1), dtype=np.float32) * (self.map.resolution / 2 * np.sqrt(2))), axis=1)
        
        # batch_robot_circles = np.concatenate((states[:, :2], np.ones(len(states), dtype=np.float32).reshape(-1, 1) * self.radius*2), axis=1)

        # dist_mat = np.sqrt(np.sum(batch_robot_circles[:, :2]**2, axis=1, keepdims=True) + np.sum(map_circles[:, :2]**2, axis=1, keepdims=True).T + (-2 * (batch_robot_circles[:, :2] @ map_circles[:, :2].T)))
        # min_dists = batch_robot_circles[:, 2].reshape(-1, 1) + map_circles[:, 2].reshape(1, -1)
        # validity_mask = dist_mat > min_dists
        # validities = np.all(validity_mask, axis=1)

        # return validities
        self.batch_size = 1000
        print(f"Num States: {len(states)}")
        

        points = self.map.get_points()
        map_circles = np.concatenate((points, np.ones((points.shape[0], 1), dtype=np.float32) * (self.map.resolution / 2 * np.sqrt(2))), axis=1)
        batch_robot_circles = np.concatenate((states[:, :2], np.ones(len(states), dtype=np.float32).reshape(-1, 1) * self.robot_radius), axis=1)
        B = batch_robot_circles.shape[0]

        stacked_validities = []
        num_batches = math.ceil(B / self.batch_size)
        for i in range(num_batches):
            print(f"Batch: {i}/{num_batches}", end='\r')
            idx_start = i * self.batch_size
            idx_end = min((i+1)*self.batch_size, B)
            validities = self.circles_to_validity(map_circles, batch_robot_circles[idx_start:idx_end])
            stacked_validities.append(validities)

        stacked_validities = np.hstack(stacked_validities)
        return stacked_validities


    
    def make_state(self, state : np.ndarray):
        return AngularNumpyState(value=state, angular_dims_start=self.angular_dims_start)

    def sample_point(self):
        width, height = self.map.get_shape_2d()
        x = np.random.uniform(0, width)
        y = np.random.uniform(0, height)
        theta = np.random.uniform(0, 2 * np.pi)
        world_coords = self.map.grid_to_approx_world_coords(np.array([x, y])) # TODO: Check if its okay to input floats to this function
        return self.make_state(np.array([*world_coords, theta]))

    def dist(self, state1, state2):
        return numpystate_distance(state1, state2)
    
    def generate_robot_representation(self, state):
        raise NotImplementedError
    
    def draw_environment(self, ax): # Kinda need this
        # raise NotImplementedError
        pass
    
    def set_obstacles(self, obstacle_set):
        raise NotImplementedError # Permanent obstacles not implemented for physical robot
    
    def batch_sample_point(self, num_points):
        raise NotImplementedError # Not Important for physical robot i think...
    
    def batch_get_robot_representations(self, states):
        return NotImplementedError # Not Important for physical robot
    
    def batch_sample_points_around_target(self, targets):
        return NotImplementedError # Not Important for physical robot

if __name__ == "__main__":
    from motion_planning.rrt import RRT
    from motion_planning.prm import PRM
    seed = np.random.randint(10000)
    print(f"Using Seed: {seed}")
    np.random.seed(seed)

    # Example usage
    # # scan = np.empty((0, 2))
    
    # mymap = Map(initial_scan=scan)
    # mymap = generate_fake_map()
    mymap = load_saved_map(directory='dumps/run1')
    # mymap.inflate_obstacles(kernel_size=3)
    mymap.visualize_points(plt.gca())
    plt.show()

    robot = PhysicalRobotSpace(mymap)
    robot.edge_validity_delta = 200.0
    rrt = RRT(env=robot)
    prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    # prm = PRM(env=robot, num_samples=10000, edge_dist_radius=500)
    prm.create_graph()
    # rrt.delta = 50.0
    rrt.delta = 100.0
    # start = robot.make_state(np.array([200.0, 500.0, 0.0]))
    # target = robot.make_state(np.array([700.0, 700.0, 0.0]))

    # start = robot.make_state(np.array([-86.0, 650.0, 0.0]))
    # target = robot.make_state(np.array([-153.0, -1300.0, np.pi/2*3]))

    # start = robot.make_state(np.array([219.0, -3850.0, 0.0]))
    # target = robot.make_state(np.array([-1135.0, -2748.0, np.pi/2*3]))

    # start = robot.make_state(np.array([-1156.0, -2415.0, 3*np.pi/2]))
    # target = robot.make_state(np.array([1278.0, -1272.0, np.pi/2]))

    start = robot.make_state(np.array([1286.0, -1288.0, 0.0]))
    target = robot.make_state(np.array([746.0, 1573.0, 0.0]))
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
    # exit()

    motion_commands = robot.path_to_motion_commands(smoothed_path)
    print(motion_commands)
    motion_commands = robot.smooth_motion_commands(motion_commands)
    print(motion_commands)
    for motion_command in motion_commands:
        print(f"Executing Motion Command: {motion_command}")
        robot.command_motion_trial(motion_command)
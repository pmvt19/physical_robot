import time
import matplotlib.pyplot as plt
import numpy as np

from motion_planning.state import NumpyState
from semantic_map import SemanticMap
from test_utils import load_saved_map, load_saved_semantic_map

from vlm_client import VLMClient
from prompts import EXTRACT_SEMANTIC_TARGETS
from vlm_output_schema import UserSemanticTarget

from robot_space import PhysicalRobotSpace
from motion_planning.prm import PRM
from particle_filter import ParticleFilter
from icp import run_icp
from utils import transformation_mat_to_state

def is_close(state1, state2, threshold=100):
    return np.linalg.norm(state1[:2] - state2[:2]) < threshold

def localize_robot(robot : PhysicalRobotSpace, pf : ParticleFilter):
    motion_commands = [['angular', 1.57],
                       ['angular', 1.57],
                       ['angular', 1.57],
                       ['angular', 1.57],]
    # motion_commands = [['angular', 1.57],]
    for motion_command in motion_commands:
        m = robot.command_motion_trial(motion_command)
        scan, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

        updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)

    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    plt.scatter(updated_state[0], updated_state[1], color='orange', zorder=2)
    plt.show()
    return updated_state

def localize_mpc_planning(robot: PhysicalRobotSpace, prm: PRM, target: NumpyState):
    pf = ParticleFilter(map_obj=robot.map)
    pf.initialize(num_particles=1000)

    # current_state = localize_robot(robot, pf)
    current_state = np.array([0.0, 0.0, 0.0])
    

    while not is_close(current_state, target.value):
        print("Replanning Path")
        path = prm.search(robot.make_state(current_state), target)
        robot.map.visualize_points(plt.gca())
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

        coords, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)
        T = run_icp(coords, robot.map.get_points(), current_state, filter_init_outliers=False, visualize=True)
        refined_state = transformation_mat_to_state(T)
        current_state = refined_state
        
        # current_state = start_state_value
        for motion_command in motion_commands:
            print(f"Executing Motion Command: {motion_command}")
            # m = robot.command_motion_trial(motion_command)
            m, predicted_state = robot.command_motion_and_predict_state(current_state, motion_command)

            coords, lidar_data = robot.read_lidar_updated(wait_for_updated_reading=True)

            # updated_state = pf.step(motion_delta=motion_command, scan=lidar_data)
            # current_state = updated_state

            # Refine the State:
            T = run_icp(coords, robot.map.get_points(), current_state, filter_init_outliers=False, visualize=False)
            refined_state = transformation_mat_to_state(T)
            current_state = refined_state
            print(f"Current State: {current_state}")

FREE_THRESHOLD = 0.5
def get_semantic_labeled_prm_vertices(semantic_map: SemanticMap, prm: PRM):
    # Get only X,Y values from vertices
    vertices = prm.graph.vertices[:, :2]

    # Convert to grid coords and get geometric map values at the coordinate positions
    grid_coord_vertices = semantic_map.batch_world_to_grid_coords(vertices)
    vertex_values = semantic_map.batch_get_value_at_grid_coords(grid_coord_vertices)

    # Mask for Vertices that fall in unoccupied cells
    unoccupied_vertices_mask = vertex_values < FREE_THRESHOLD

    # Filtering to vertices that fall in unoccupied cells
    unoccupied_vertices = vertices[unoccupied_vertices_mask]

    # Converting unoccupied vertices to grid coords
    unoccupied_vertices_grid_coords = semantic_map.batch_world_to_grid_coords(unoccupied_vertices)

    # Get Flood Filled Semantic Values for vertices in unoccupied cells
    unoccupied_vertices_semantics = semantic_map.batch_get_semantic_flood_fill_value_at_grid_coords(unoccupied_vertices_grid_coords)

    # Mask for Unoccupied Vertices which have Semantic Labels
    unoccupied_vertices_labeled_semantics_mask = np.sum(unoccupied_vertices_semantics, axis=1) > 0

    # Get unoccupied vertices that have labeled semantic (not the semantics)
    unoccupied_vertices_with_labeled_semantics = unoccupied_vertices[unoccupied_vertices_labeled_semantics_mask]

    # Get unoccupied vertices semantics (not the coords)
    unoccupied_vertices_labeled_semantics = unoccupied_vertices_semantics[unoccupied_vertices_labeled_semantics_mask]

    return unoccupied_vertices_with_labeled_semantics, unoccupied_vertices_labeled_semantics

def get_target_from_semantics(robot: PhysicalRobotSpace, semantic_map: SemanticMap, vertices: np.ndarray, semantics: np.ndarray, layer: str, item: str, target_theta: float = 0.0):

    # Filter Semantics to only the layer of interest: room or object
    layer_semantics = semantics[:, semantic_map.layer_name_to_idx[layer]]

    # Get Item Id of item
    item_id = -1
    if layer == 'room':
        item_id = semantic_map.room_to_id[item]
    elif layer == 'object':
        item_id = semantic_map.object_to_id[item]
    else:
        raise NotImplementedError

    # Create Vertices Mask for Vertices Corresponding to Specificed item
    selected_item_mask = layer_semantics == item_id

    # Get Vertices Corresponding to Specified Item
    selected_item_vertices = vertices[selected_item_mask]

    # Randomly Choose a Vertex from the list of remaining options
    target_pos_idx = np.random.choice(len(selected_item_vertices))
    target_pos = selected_item_vertices[target_pos_idx]

    # Create Robot State
    target = robot.make_state(np.array([target_pos[0], target_pos[1], target_theta]))

    return target


def run_semantic_motion_planning(map_save_dir):
    semantic_map: SemanticMap = load_saved_semantic_map(directory=map_save_dir)
    semantic_map.resolution = 10 # TODO REMOVE
    # Flood Fill Map
    semantic_map.flood_fill(limit_fill_extent=False, method='bfs')

    # Create RobotSpace
    robot = PhysicalRobotSpace(semantic_map)
    robot.edge_validity_delta = 200.0 # TODO REMOVE

    # Initialize and Create PRM
    prm = PRM(env=robot, num_samples=20000, num_neighbors=10, validate_edges=True)
    prm.create_graph()

    semantic_map.print_item_ids()

    visualize = True
    if visualize:
        # Visualize Map Layers for Verification
        semantic_map.visualize(plt.gca())
        plt.show()

        fig, ax = plt.subplots(1, 2)
        semantic_map.visualize_semantic_layer(ax[0], layer='room')
        semantic_map.visualize_semantic_layer(ax[1], layer='object')
        plt.show()

        # Visualize Flood Fill Map
        fig, ax = plt.subplots(1, 2)
        semantic_map.visualize_flood_fill_layer(ax[0], layer='room')
        semantic_map.visualize_flood_fill_layer(ax[1], layer='object')
        plt.show()

    semantic_vertices, semantic_labels = get_semantic_labeled_prm_vertices(semantic_map, prm)
    target = get_target_from_semantics(robot, semantic_map, semantic_vertices, semantic_labels, 'object', 'cardboard')

    start = robot.make_state(np.array([0.0, 0.0, 0.0]))
    path = prm.search(start, target)

    semantic_map.visualize_points(plt.gca())
    prm.draw(plt.gca(), path=path, show_task=True)
    plt.show()
    
def get_semantic_target_from_user(vlm_client: VLMClient, semantic_map: SemanticMap):
    while True:
        user_input = input("Please provide where you want the robot to travel (object or room)\n")
        vlm_response = vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format(user_input))
        user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response.text)

        if user_semantic_target.valid:
            break
        else:
            print("Unable to extract semantic information from input. Please Try Again!\n")
    
    return user_semantic_target.semantic_level, user_semantic_target.item_name

if __name__ == '__main__':
    # run_semantic_motion_planning(map_save_dir='saves/scenes/extensive_apartment')

    vlm_client = VLMClient()
    print(get_semantic_target_from_user(vlm_client))
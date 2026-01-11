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
from robot_utils import localize_robot, mpc_plan_and_follow_trajectory

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

def get_target_from_semantics(robot: PhysicalRobotSpace, 
                              semantic_map: SemanticMap, 
                              vertices: np.ndarray, 
                              semantics: np.ndarray, 
                              layer: str, 
                              item: str, 
                              target_theta: float = 0.0):

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
    
def get_semantic_target_from_user(vlm_client: VLMClient, semantic_map: SemanticMap):
    while True:
        user_input = input("Please provide where you want the robot to travel (object or room)\n")
        vlm_response = vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format(user_input, 
                                                                             semantic_map.get_room_list(),
                                                                             semantic_map.get_object_list(),
                                                                             semantic_map.get_invalid_object_list()))
        user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response.text)

        if user_semantic_target.valid:
            break
        else:
            print(f"Unable to extract semantic information from input.\n \
                  Reasoning:\n{user_semantic_target.reason} \nPlease Try Again!\n")
    print(user_semantic_target.reason)
    return user_semantic_target.semantic_level, user_semantic_target.item_name

def run_semantic_motion_planning(robot: PhysicalRobotSpace):
    # Initialize VLM Client
    vlm_client = VLMClient()

    # Get Map from Robot
    semantic_map: SemanticMap = robot.map
    semantic_map.resolution = 10 # TODO: Hack for now remove this

    # Flood Fill the Map
    semantic_map.flood_fill(limit_fill_extent=False, method='bfs')

    # Get Semantic Target Layer and Item from User Natural Language Input
    semantic_layer, item = get_semantic_target_from_user(vlm_client, semantic_map)

    # Create and Build PRM
    prm = PRM(env=robot, num_samples=10000, num_neighbors=10, validate_edges=True)
    prm.create_graph()

    # Label Vertices in PRM with Semantic Information
    semantic_vertices, semantic_labels = get_semantic_labeled_prm_vertices(semantic_map, prm)

    # Get Target State Based on User Semantic Input 
    target = get_target_from_semantics(robot, semantic_map, semantic_vertices, semantic_labels, semantic_layer.lower(), item)

    # Localize Robot within Map
    start, pf = localize_robot(robot, semantic_map)

    # Plan and Follow path with MPC
    mpc_plan_and_follow_trajectory(robot, pf, robot.map, prm, robot.make_state(start), target)


if __name__ == '__main__':
    # Load Advanced Map
    semantic_map = load_saved_semantic_map(directory='saves/scenes/extensive_apartment')
    
    # Initialize Robot Space
    robot = PhysicalRobotSpace(semantic_map)

    # Run Semantic Motion Planning Algorithm
    run_semantic_motion_planning(robot)
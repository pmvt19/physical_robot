import numpy as np

from robot import Robot
from map import Map
from semantic_map import SemanticMap
from robot_utils import localize_robot, mpc_plan_and_follow_trajectory
from robot_space import PhysicalRobotSpace
from motion_planning.prm import PRM
from vlm_client import VLMClient

class MotionPlanner():
    def __init__(self, robot: Robot, map: Map):
        self.robot: Robot = robot
        self.map: Map = map

        self.robot_space: PhysicalRobotSpace = PhysicalRobotSpace(map_obj=self.map)

    def localize_robot(self):
        self.start, self.pf = localize_robot(self.robot, self.map)

    def build_prm(self):
        # Create and Build PRM
        self.prm = PRM(env=self.robot_space, num_samples=10000, num_neighbors=10, validate_edges=True)
        self.prm.create_graph()

    def get_target_from_user(self):
        # TODO: Implement robust function for user to tell where the robot should go
        return None

    def run_motion_planning(self):
        # Localize the Robot
        self.localize_robot()

        # Get Target From User
        target = self.get_target_from_user()

        # Convert Target to NumpyState
        target = self.robot_space.make_state(target)

        # Plan a path to the Target and Follow with MPC
        mpc_plan_and_follow_trajectory(self.robot_space, self.pf, self.map, self.prm, self.robot_space.make_state(self.start), target)

FREE_THRESHOLD = 0.5
class SemanticMotionPlanner(MotionPlanner):
    def __init__(self, robot: Robot, semantic_map: SemanticMap):
        assert (isinstance(semantic_map, SemanticMap))
        self.robot: Robot = robot
        self.map: SemanticMap = semantic_map

        self.robot_space: PhysicalRobotSpace = PhysicalRobotSpace(map_obj=self.map)

        self.vlm_client = VLMClient()

    def get_semantic_labeled_prm_vertices(self):
        # Get only X,Y values from vertices
        vertices = self.prm.graph.vertices[:, :2]

        # Convert to grid coords and get geometric map values at the coordinate positions
        grid_coord_vertices = self.map.batch_world_to_grid_coords(vertices)
        vertex_values = self.map.batch_get_value_at_grid_coords(grid_coord_vertices)

        # Mask for Vertices that fall in unoccupied cells
        unoccupied_vertices_mask = vertex_values < FREE_THRESHOLD

        # Filtering to vertices that fall in unoccupied cells
        unoccupied_vertices = vertices[unoccupied_vertices_mask]

        # Converting unoccupied vertices to grid coords
        unoccupied_vertices_grid_coords = self.map.batch_world_to_grid_coords(unoccupied_vertices)

        # Get Flood Filled Semantic Values for vertices in unoccupied cells
        unoccupied_vertices_semantics = self.map.batch_get_semantic_flood_fill_value_at_grid_coords(unoccupied_vertices_grid_coords)

        # Mask for Unoccupied Vertices which have Semantic Labels
        unoccupied_vertices_labeled_semantics_mask = np.sum(unoccupied_vertices_semantics, axis=1) > 0

        # Get unoccupied vertices that have labeled semantic (not the semantics)
        unoccupied_vertices_with_labeled_semantics = unoccupied_vertices[unoccupied_vertices_labeled_semantics_mask]

        # Get unoccupied vertices semantics (not the coords)
        unoccupied_vertices_labeled_semantics = unoccupied_vertices_semantics[unoccupied_vertices_labeled_semantics_mask]

        return unoccupied_vertices_with_labeled_semantics, unoccupied_vertices_labeled_semantics
    
    def get_target_from_semantics(self,
            robot: PhysicalRobotSpace, 
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
        print(f"Assigned Target State: {np.round(target.value, 2)}")
        return target
    
    def get_semantic_target_from_user(self):
        while True:
            user_input = input("Please provide where you want the robot to travel (object or room)\n")
            vlm_response = self.vlm_client.text_query(EXTRACT_SEMANTIC_TARGETS.format(user_input, 
                                                                                self.map.get_room_list(),
                                                                                self.map.get_object_list(),
                                                                                self.map.get_invalid_object_list()), 
                                                                                UserSemanticTarget.model_json_schema())
            user_semantic_target = UserSemanticTarget.model_validate_json(vlm_response)

            if user_semantic_target.valid:
                break
            else:
                print(f"Unable to extract semantic information from input.\n \
                    Reasoning:\n{user_semantic_target.reason} \nPlease Try Again!\n")
        print(user_semantic_target.reason)
        return user_semantic_target.semantic_level, user_semantic_target.item_name

    def run_motion_planning(self):
        # Flood Fill the Semantic Map
        self.map.flood_fill(limit_fill_extent=False, method='bfs')

        # Get Semantic Target Layer and Item from User Natural Language Input
        semantic_layer, item = self.get_semantic_target_from_user()

        # Create and Build PRM
        self.build_prm()

        # Label Vertices in PRM with Semantic Information
        semantic_vertices, semantic_labels = self.get_semantic_labeled_prm_vertices()

        # Get Target State Based on User Semantic Input 
        target = self.get_target_from_semantics(semantic_vertices, semantic_labels, semantic_layer.lower(), item)

        # Localize Robot within Map
        self.localize_robot()

        # Plan and Follow path with MPC
        mpc_plan_and_follow_trajectory(self.robot_space, self.pf, self.map, self.prm, self.robot_space.make_state(self.start), target)

import numpy as np
from physical_robot.maps import Map, SemanticMap
# from semantic_map import SemanticMap

import pickle

def generate_fake_map():
    xs_1 = np.linspace(0, 1000, 10000)
    ys_1 = np.ones_like(xs_1) * 0.0

    ys_2 = np.linspace(0, 2000, 20000)
    xs_2 = np.ones_like(ys_2) * 1000.0

    xs_3 = np.linspace(0, 1000, 10000)
    ys_3 = np.ones_like(xs_3) * 2000.0

    ys_4 = np.linspace(0, 2000, 20000)
    xs_4 = np.ones_like(ys_4) * 0.0

    s1 = np.stack((xs_1, ys_1), axis=1)
    s2 = np.stack((xs_2, ys_2), axis=1)
    s3 = np.stack((xs_3, ys_3), axis=1)
    s4 = np.stack((xs_4, ys_4), axis=1)

    init_points = np.vstack((s1, s2, s3, s4))

    mymap = Map()
    mymap.init_map(init_points)
    return mymap

def generate_fake_scan():
    xs_1 = np.linspace(-1000, 1000, 90)
    ys_1 = np.ones_like(xs_1) * -1000.0

    ys_2 = np.linspace(-1000, 1000, 90)
    xs_2 = np.ones_like(ys_2) * 1000.0

    xs_3 = np.linspace(-1000, 1000, 90)
    ys_3 = np.ones_like(xs_3) * 1000.0

    ys_4 = np.linspace(-1000, 1000, 90)
    xs_4 = np.ones_like(ys_4) * -1000.0

    s1 = np.stack((xs_1, ys_1), axis=1)
    s2 = np.stack((xs_2, ys_2), axis=1)
    s3 = np.stack((xs_3, ys_3), axis=1)
    s4 = np.stack((xs_4, ys_4), axis=1)

    init_points = np.vstack((s1, s2, s3, s4))
    return init_points

def pseudolabel_map(semantic_map : SemanticMap):
    # semantic_map.visualize(plt.gca())
    semantic_map.map.visualize_points(plt.gca())
    plt.show()

    # Inject Fake Semantic Labels
    map_points = semantic_map.map.get_points() # (9312, 2) for apartment labels
    # print(type(map_points), map_points.shape)
    # exit()

    label_values = np.zeros((map_points.shape[0],)).astype(np.int32)
    office_label_mask = np.logical_and(map_points[:, 0] < -1514, map_points[:, 1] > 1000)
    label_values[office_label_mask] = 1

    dining_room_label_mask = np.logical_and(map_points[:, 0] > -729, map_points[:, 1] > 809)
    label_values[dining_room_label_mask] = 2

    kitchen_label_mask = np.logical_and(map_points[:, 0] > 1249, map_points[:, 1] > -1200)
    label_values[kitchen_label_mask] = 3

    room_label_mask = np.logical_and(map_points[:, 0] < -655, map_points[:, 1] < -1241)
    label_values[room_label_mask] = 4

    entrance_label_mask = np.logical_and(map_points[:, 0] > 859, map_points[:, 1] < -4113)
    label_values[entrance_label_mask] = 5
    
    grid_coords = semantic_map.map.batch_world_to_grid_coords(map_points)

    semantic_map.semantic_layer[grid_coords[:, 0], grid_coords[:, 1], 0] = label_values
    plt.imshow(np.rot90(semantic_map.semantic_layer[:, :, 0]))
    plt.show()

def load_saved_map(directory='saves/scenes/tmp'):
    mymap = pickle.load(open(f"{directory}/map/map_object_final.pickle", "rb"))
    return mymap

def load_saved_advanced_map(directory='saves/scenes/extensive_apartment'):
    advanced_map = pickle.load(open(f"{directory}/advanced_map/advanced_map_object_final.pickle", "rb"))
    return advanced_map

def load_saved_semantic_map(directory='saves/scenes/advanced_semantic_apartment'):
    semantic_map = pickle.load(open(f"{directory}/semantic_map/semantic_map_object_final.pickle", "rb"))
    return semantic_map

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    semantic_map = load_saved_semantic_map()
    fig, ax = plt.subplots(1, 3)
    semantic_map.visualize(ax, visualize_layers=True)
    plt.show()

    mymap = load_saved_map()
    mymap.visualize(plt.gca())
    plt.show()

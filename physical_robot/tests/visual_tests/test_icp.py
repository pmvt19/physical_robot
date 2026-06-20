import unittest
import numpy as np

from physical_robot.algorithms.icp import run_icp
# Note this file will not be considered a Unit test as we will only verify
# correctness visually

class TestICP(unittest.TestCase):
    # TEST 1: ICP with only rotation
    def test_icp_rotation(self):
        coords_0 = np.load("test_data/icp/icp_sample_data_rotation_0.npy")
        coords_1 = np.load("test_data/icp/icp_sample_data_rotation_1.npy")
        state_1 = np.load("test_data/icp/state_rotation_1.npy")

        run_icp(coords_1, coords_0, state_1, visualize=True)

    # TEST 2: ICP with only translation
    def test_icp_translation(self):
        coords_0 = np.load("test_data/icp/icp_sample_data_translational_0.npy")
        coords_1 = np.load("test_data/icp/icp_sample_data_translational_1.npy")
        state_1 = np.load("test_data/icp/state_translational_1.npy")

        run_icp(coords_1, coords_0, state_1, visualize=True)

    # TEST 3: ICP with rotation and translation
    def test_icp_rotation_and_translation(self):
        coords_0 = np.load("test_data/icp/icp_sample_data_rotation_and_translation_0.npy")
        coords_4 = np.load("test_data/icp/icp_sample_data_rotation_and_translation_4.npy")
        state_4 = np.load("test_data/icp/state_rotation_and_translation_4.npy")

        run_icp(coords_4, coords_0, state_4, visualize=True)

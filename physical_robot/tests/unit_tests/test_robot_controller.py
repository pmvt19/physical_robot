import unittest
import numpy as np

from physical_robot.robot.robot_controller import RobotController

# Testing Unittest

class RobotControllerTests(unittest.TestCase):
    def test_unit_test(self):
        fake_path = [
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 10.0, np.pi]),
                np.array([10.0, 10.0, np.pi/2]),
                np.array([10.0, 0.0, 0.0]),
                np.array([0.0, 0.0, np.pi*1.5]),
            ]
        
        controller = RobotController()
        print(controller.compute_motion_commands(fake_path))
    
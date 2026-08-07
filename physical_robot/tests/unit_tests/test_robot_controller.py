import unittest

import numpy as np

from physical_robot.robot.robot_controller import RobotController


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
        motion_commands = controller.compute_motion_commands(fake_path)

        expected_motion_commands = [
            ('angular', np.float64(1.5707963267948966)),
            ('linear', np.float64(10.0)),
            ('angular', np.float64(-1.5707963267948966)),
            ('linear', np.float64(10.0)),
            ('angular', np.float64(-1.5707963267948966)),
            ('linear', np.float64(10.0)),
            ('angular', np.float64(-1.5707963267948968)),
            ('linear', np.float64(10.0)),
            ('angular', np.float64(1.5707963267948966))
        ]

        self.assertListEqual(expected_motion_commands, motion_commands)
    
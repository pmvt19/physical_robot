import numpy as np

class RobotController():
    def __init__(self):
        pass

    def compute_angular_difference(self, start_theta, target_theta):
        angular_diff = target_theta - start_theta
        return np.arctan2(np.sin(angular_diff), np.cos(angular_diff))

    def state_pairs_to_motion_commands(self, state1, state2):
        x1, y1, theta1 = state1
        x2, y2, theta2 = state2

        state1_translational = state1[:2]
        state2_translational = state2[:2]

        translation_diff = state2_translational - state1_translational

        translational_heading = np.arctan2(y2-y1, x2-x1)
        linear_dist = np.linalg.norm(translation_diff)

        translational_heading
        translational_angular_delta = self.compute_angular_difference(theta1, translational_heading)
        match_target_heading_angular_delta = self.compute_angular_difference(translational_heading, theta2)

        commands = [
            ("angular", translational_angular_delta),
            ("linear", linear_dist),
            ("angular", match_target_heading_angular_delta)
        ]

        return commands
    
    def smooth_motion_commands(self, commands):
        """
        Insert Logic for Smoothing Here
        
        :param self: Description
        :param commands: Description
        """
    
        # Handle Base Case with No Commands
        if len(commands) == 0:
            return []
        
        smoothed_commands = []
        idx = 0
        while idx < len(commands)-1:
            command1 = commands[idx]
            command2 = commands[idx+1]

            if command1[0] == 'angular' and command2[0] == 'angular':
                summed_angular_deltas = command1[1] + command2[1]
                smoothed_angular_delta = np.arctan2(np.sin(summed_angular_deltas), np.cos(summed_angular_deltas))
                smoothed_commands.append(("angular", smoothed_angular_delta))
                idx += 1
            else:
                smoothed_commands.append(command1)
            
            idx += 1
        smoothed_commands.append(commands[-1])

        return smoothed_commands


    def compute_motion_commands(self, path: list[np.ndarray]):
        commands = []
        for i in range(len(path)-1):
            state1 = path[i]
            state2 = path[i+1]
            commands.extend(self.state_pairs_to_motion_commands(state1, state2))

        smoothed_commands = self.smooth_motion_commands(commands)
        return smoothed_commands
            


if __name__ == '__main__':
    ## CONVERT TO ROBOT CONTROLLER TEST 
    fake_path = [
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 10.0, np.pi]),
        np.array([10.0, 10.0, np.pi/2]),
        np.array([10.0, 0.0, 0.0]),
        np.array([0.0, 0.0, np.pi*1.5]),
    ]

    controller = RobotController()
    print(controller.compute_motion_commands(fake_path))
    
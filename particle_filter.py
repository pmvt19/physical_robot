import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from scipy import ndimage, stats
from map import Map

from simulate_lidar import SimulatedLidar

from utils import pairwise_dists
from test_utils import generate_fake_map, load_saved_map

class ParticleFilter():
    def __init__(self, map_obj, scale_factor=50):
        self.map : Map = map_obj
        self.scale_factor = scale_factor
        print("WARNING: GUESS ON THE SCALE FACTOR!!")

        self.normal_distribution = stats.norm(loc=0, scale=self.scale_factor)

    def _compute_dist_map(self):
        inverse_map = 1 - self.map.map
        self.dist_map = ndimage.distance_transform_edt(inverse_map)

    def _batch_get_simulated_lidar(self):
        raise NotImplementedError

    def _batch_get_probabilities(self):
        raise NotImplementedError
    
    def generate_initial_particles(self, num_particles):
        print("WARNING: GENERATE INNITIAL PARTICLES NOT WORKING AS INTENDED!!!")
        self.particles = np.random.uniform(low=np.array([-2000, -2000, 0]), high=np.array([2000, 3000, 2*np.pi]), size=(num_particles, 3))
        weights = 1 / num_particles
        batch_uniform_weights = np.ones(num_particles) * weights
        self.particles = np.concatenate((self.particles, batch_uniform_weights.reshape(-1, 1)), axis=1)

    # TODO: Change function name - Done?
    def batch_get_measurement_update(self, states, scan_actual, tmp_points=None):
        # States: (N, 3)
        # Scan_Actual: (M, 2)
        # Ideally, scan_actual[i] is just (angle, dist)

        ### ---- Get Batch Simulated Lidar ---- ###
        N, d = states.shape

        angles = scan_actual[:, 0] # (M,)
        point_dists = scan_actual[:, 1] # (M,)

        state_headings = states[:, 2] # (N,)

        ### TODO: CHECK THIS: TODO ###
        offset_angles = angles.reshape(-1, 1) - (np.pi/2 - state_headings.reshape(1, -1)) # (M, 1) + (1, N) = (M, N)
        ### TODO: CHECK THIS: TODO ###

        # print(np.rad2deg(angles % (2*np.pi)), state_headings)
        # exit()

        coses = np.cos(offset_angles) # (M, N)
        sines = np.sin(offset_angles) # (M, N)
        vecs = np.stack((coses, sines), axis=2) # (M, N, 2)

        origin_centered_points = vecs * point_dists.reshape(-1, 1, 1) # (M, N, 2) * (M,) = (M, N, 2) (MIGHT NEED TO RESHAPE point_dists)
        batch_simulated_lidar_readings = origin_centered_points + states[:, :2].reshape(1, N, -1) # (M, N, 2) + (1, N, 2) = (M, N, 2) (Probably need to transform states a bit)
        batch_simulated_lidar_readings = batch_simulated_lidar_readings.transpose(1, 0, 2) # Transpose the Matrix to be (N, M, 2) {I think this makes more sense, but I already implemented this function...}

        # plt.scatter(batch_simulated_lidar_readings[0, :, 0], batch_simulated_lidar_readings[0, :, 1], label='sim lidar')
        # plt.scatter(tmp_points[:, 0], tmp_points[:, 1], label='scanned lidar')
        # self.map.visualize_points(plt.gca())
        # plt.legend()
        # plt.show()
        ### ---- Get Batch Simulated Lidar ---- ###

        ### ---- Get Probabilities ---- ###
        flattened_batch_simulated_lidar_readings = batch_simulated_lidar_readings.reshape(-1, 2) # (N, M, 2) -> (N*M, 2)
        flattened_batch_grid_coords = self.map.batch_world_to_grid_coords(flattened_batch_simulated_lidar_readings) # (N*M, 2)
        flattened_batch_dists = self.dist_map[flattened_batch_grid_coords[:, 0], flattened_batch_grid_coords[:, 1]] # (N*M,)
        print(flattened_batch_dists)
        flattened_batch_probs = self.normal_distribution.pdf(flattened_batch_dists) # (N*M,)
        batch_probs = flattened_batch_probs.reshape(N, -1) # (N, M)
        ### ---- Get Probabilities ---- ###

        ### ---- Add Noise??? ---- ###

        ### ---- Add Noise??? ---- ###

        ### ---- Get Particle Weights ---- ###

        # -- Should have an Underflow Problem -- #
        # batch_weights_unnormalized = np.prod(batch_probs, axis=0).squeeze()
        # batch_weights_normalized = batch_weights_unnormalized / np.sum(batch_weights_unnormalized)
        # -- Should have an Underflow Problem -- #


        # -- Should avoid Underflow Problem with Log Scaling -- #
        print("batch probs shape", batch_probs.shape)
        batch_log_probs = np.log(batch_probs)
        batch_log_weights = np.sum(batch_log_probs, axis=1).squeeze()
        batch_rescaled_log_weights = batch_log_weights - np.max(batch_log_weights)
        batch_unnormalized_weights = np.exp(batch_rescaled_log_weights)
        batch_normalized_weights = batch_unnormalized_weights / np.sum(batch_unnormalized_weights)
        # -- Should avoid Underflow Problem with Log Scaling -- #

        ### ---- Get Particle Weights ---- ###



        return batch_simulated_lidar_readings, batch_probs, batch_normalized_weights

    def _noise_injection(self, particles):

        mu_noise = np.array([0.0, 0.0, 0.0])
        Q_noise = np.array([3.0, 3.0, 0.2])

        # noise = np.random.normal(loc=mu_noise, scale=Q_noise, size=(len(particles), 3))
        noise = np.random.normal(loc=mu_noise, scale=np.array([10.0, 10.0, 0.5]), size=(len(particles), 3))
        particles[:, :3] = particles[:, :3] + noise
        return particles
    
    def _delta_noise_injection(self, motion_delta):
        num_particles = len(self.particles)
        # delta_noise = np.random.normal(loc=np.array([0.0, 0.0, 0.0]), scale=np.array([0.1, 0.1, 0.1]), size=(num_particles, 3))
        # delta_noise = np.random.normal(loc=np.array([0.0, 0.0, 0.0]), scale=np.array([10.0, 10.0, 0.1]), size=(num_particles, 3)) # Works!!!
        delta_noise = np.random.normal(loc=np.array([0.0, 0.0, 0.0]), scale=np.array([2.0, 2.0, 0.1]), size=(num_particles, 3))
        noisy_motion_delta = motion_delta + delta_noise
        return noisy_motion_delta

    def resample(self, num_particles_to_sample=1000):
        # Resampling Implicitly Handles Killing Low-Weight Particles
        num_particles = len(self.particles)
        particle_weights = self.particles[:, 3]

        num_particles_to_sample = num_particles
        sampled_particle_idxes = np.random.choice(num_particles, p=particle_weights, size=num_particles_to_sample, replace=True)

        sampled_particles = self.particles[sampled_particle_idxes]
        sampled_particles_noisy = self._noise_injection(sampled_particles)

        # Reset Weights to Uniform Distribution
        sampled_particles_noisy[:, 3] = (1/num_particles_to_sample)

        # Update Particles
        self.particles = sampled_particles_noisy

    def get_state_estimate(self, method = 'MLE'):
        if method == 'MLE':
            positional_states = self.particles[:, :2]
            orientation_states = self.particles[:, 2]
            particle_weights = self.particles[:, 3]

            mean_position = np.sum(particle_weights.reshape(-1, 1) * positional_states, axis=0)

            coses = np.cos(orientation_states)
            sines = np.sin(orientation_states)

            proxy_x_theta = np.sum(particle_weights * coses)
            proxy_y_theta = np.sum(particle_weights * sines)

            mean_theta = np.arctan2(proxy_y_theta, proxy_x_theta)

            # TODO: I don't like this, use concatentation
            return np.array([mean_position[0], mean_position[1], mean_theta])
        elif method == 'MAP':
            particle_weights = self.particles[:, 3]
            best_estimate_idx = np.argmax(particle_weights)

            # Not sure if copying here is necessary
            best_estimate = np.copy(self.particles[best_estimate_idx, :3])
            return best_estimate
        else:
            raise Exception('State Estimation Must Use Either MLE or MAP Estimation Method')

    def get_updated_particles(self, motion_delta):
        noisy_motion_delta = self._delta_noise_injection(motion_delta)
        self.particles[:, :3] = self.particles[:, :3] + noisy_motion_delta
        return self.particles
        
    def update_particle_weights(self, scan):
        _, _, batch_normalized_weights = self.batch_get_measurement_update(self.particles[:, :3], scan)
        print(self.particles.shape, batch_normalized_weights.shape)
        self.particles[:, 3] = batch_normalized_weights

    def step(self, motion_delta, scan, estimate_method='MLE'):
        self.particles = self.get_updated_particles(motion_delta)
        self.update_particle_weights(scan)
        state_estimate = self.get_state_estimate(method=estimate_method)
        self.resample()
        return state_estimate
    
    ### --- Visualization Functions --- ###

    def visualize_dist_map(self, ax):
        ax.imshow(np.rot90(self.dist_map))
    
    def visualize_map(self, ax):
        self.map.visualize(ax)

    def visualize_particles(self, ax, use_grid_coords=False):
        if use_grid_coords:
            grid_coords = self.map.world_to_grid_coords(self.particles[:, :2])
            ax.scatter(grid_coords[:, 0], grid_coords[:, 1], color='blue')
        else:
            ax.scatter(self.particles[:, 0], self.particles[:, 1], color='blue')
        angles = self.particles[:, 2]

        coses = np.cos(angles)
        sines = np.sin(angles)
        vecs = np.stack((coses, sines), axis=1) * 100

        header_vec_eps = vecs + self.particles[:, :2]
        # LineCollection to do this
        # TODO: Check this
        num_particles = len(self.particles)
        lines = [(self.particles[i, :2], header_vec_eps[i]) for i in range(num_particles)]
        ax.add_collection(LineCollection(lines, color="red", alpha=0.5))

if __name__ == '__main__':
    mymap = generate_fake_map()
    # mymap = load_saved_map()

    state = np.array([-1000.0, 2500.0, np.pi/2])

    ### ---- Used for Testing ---- ###
    # state = np.array([-1000.0, 2500.0, np.pi/4])
    # state = np.array([-1000.0, 2500.0, 3*np.pi/4])
    # state = np.array([-1000.0, 2500.0, 3*np.pi/2])
    # state = np.array([-1000.0, 2500.0, 5*np.pi/2])
    # state = np.array([-1000.0, 2500.0, 0])

    sl = SimulatedLidar(mymap, 100, 10000)
    translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=state)
    # Format Lidar Readings:

    # print(translated_rotated_points.shape, line_segment_eps.shape, map_points.shape)
    # print(angles.shape, r_dists.shape)
    # scan = np.stack((angles, r_dists), axis=1)
    # scan_v2 = np.stack((r_angles_local, r_dists), axis=1)
    # print(np.rad2deg(angles))
    # print(np.rad2deg(r_angles_local) % 360)
    # # exit()
    # print("Scan Shapes")
    # print(scan.shape, scan_v2.shape)


    pf = ParticleFilter(mymap)
    pf._compute_dist_map()
    pf.generate_initial_particles(num_particles=10000)

    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    plt.scatter(state[0], state[1], color='orange')
    plt.show()

    # fig, (ax1, ax2) = plt.subplots(1, 2)
    # pf.visualize_map(ax1)
    # pf.visualize_dist_map(ax2)
    # plt.show()
    
    
    # batch_simulated_lidar_readings, batch_probs, batch_normalized_weights =\
    #     pf.batch_get_measurement_update(state.reshape(-1, 3), scan_v2, translated_rotated_points)
    # print(batch_probs)

    motion_delta = np.array([0.0, 100.0, 0.0])

    new_state = np.array([-1000.0, 2600, np.pi/2])
    translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=new_state)
    scan_v2 = np.stack((r_angles_local, r_dists), axis=1)
    state_estimate = pf.step(motion_delta=motion_delta, scan=scan_v2)
    print(f"State Estiamte: {np.round(state_estimate, 2)}")

    pf.visualize_particles(plt.gca())
    pf.map.visualize_points(plt.gca())
    plt.scatter(new_state[0], new_state[1], color='orange')
    plt.show()

    mds = [
        [0.0, -100.0, 0.0] 
    ] * 45 + [
        [100.0, 0.0, 0.0]
    ] * 30 + [
        [0.0, 100.0, 0.0]
    ] * 40

    for i in range(len(mds)):
        motion_delta = np.array(mds[i])

        new_state = new_state + motion_delta
        translated_rotated_points, line_segment_eps, map_points, angles, r_dists, unrotated_points, r_angles_local = sl.simulate_lidar(loc=new_state)
        scan_v2 = np.stack((r_angles_local, r_dists), axis=1)
        state_estimate = pf.step(motion_delta=motion_delta, scan=scan_v2)
        print(f"Actual State: {np.round(new_state, 2)}")
        print(f"State Estimate: {np.round(state_estimate, 2)}")

        pf.visualize_particles(plt.gca())
        pf.map.visualize_points(plt.gca())
        plt.scatter(new_state[0], new_state[1], color='orange', zorder=2)
        plt.show()
    


    
        
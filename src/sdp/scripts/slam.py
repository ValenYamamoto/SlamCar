import math
import numpy as np
from typing import List, Tuple, Callable

from utils import Particle, ParametricLine, get_intersection_with_map, Landmark, pi_2_pi

def print_observations(z):
    for i in range(z.shape[1]):
        print(f"Angle: {z[1, i]} Dist: {z[0,i]}")

class SLAMContext:
    constants = {
        "Q": np.diag([5.0, np.deg2rad(0.75)]) ** 2,
        "R": np.diag([1.0, np.deg2rad(0.5)]) ** 2,
        "DELTA": 1,
        "N_PARTICLES": 20,
        "N_LANDMARKS": 2,
        "LANDMARK_R": 1.5,
        "LM_THRESH": 2,
        "ANGLES": [-1.5708, -1.0472, -0.5236, 0, 0.5236, 1.0472, 1.5708],
        "RANGE": 10,
        'x': 0,
        'y': 0,
        'theta': 0,
        'WALLS_X': 0,
        'WALLS_Y': 0,
        'LANDMARK_X': [],
        'LANDMARK_Y': [],
        'WALLS_Y': 0,
        'MOVES': [],
        'RATE': [],
        'X1': 0,
        'X2': 0,
    }

    @classmethod
    def init_from_dict(cls, d):
        a = cls()
        for k, v in a.constants.items():
            if k in d:
                a[k] = d[k]
        return a

    def __getitem__(self, idx: int):
        if idx in self.constants:
            return self.constants[idx]
        raise ValueError

    def __setitem__(self, idx: int, value):
        self.constants[idx] = value

    def __str__(self):
        return str(self.constants)


class FastSLAM:
    def __init__(
        self,
        ctx: SLAMContext,
        x: float,
        y: float,
        theta: float,
        motion_model: Callable,
        map_lines: List[ParametricLine],
    ):
        self.motion_model = motion_model
        # self.get_observations = get_observations
        self.particles = [Particle(ctx, x, y, theta) for i in range(ctx["N_PARTICLES"])]
        self.map_lines = map_lines
        self.ctx = ctx

    def add_new_landmark(
        self, particle: Particle, z: np.array, Q: np.array
    ) -> Particle:
        """
        Add new landmark to particle
        """
        r = z[0]
        theta = z[1]
        landmark_idx = int(z[2])

        ### TODO: actually check
        s = math.sin(pi_2_pi(particle.orientation() + theta))
        c = math.cos(pi_2_pi(particle.orientation() + theta))
        particle.landmarks[landmark_idx].mu[0] = particle.x() + r * s
        particle.landmarks[landmark_idx].mu[1] = particle.y() + r * c
        #print(theta, r, particle.x(), particle.y(), particle.orientation())

        Gz = np.array([[c, -r * s], [s, r * c]])

        particle.landmarks[landmark_idx].sigma = Gz @ Q @ Gz.T

        return particle

    def compute_MC_expected_position(self):
        total_x, total_y, total_orientation = 0, 0, 0
        for particle in self.particles:
            total_x += particle.x()
            total_y += particle.y()
            total_orientation += particle.orientation()
        x = total_x / self.ctx["N_PARTICLES"]
        y = total_y / self.ctx["N_PARTICLES"]
        o = total_orientation / self.ctx["N_PARTICLES"]
        return np.array([[x, y, o]]).T

    def compute_weight(self, particle: Particle, z: np.array, Q: np.array) -> float:
        """
        computes the importance weight for a single particle for a single landmark

        args:
        particle - Particle for which to compute weight for
        z - observation of landmark for the current time step
        Q - measurement covariance, as np array

        returns:
        w - the importance weight for this landmark
        """
        landmark_idx = int(z[2])
        landmark = particle.landmarks[landmark_idx]
        zp, Hv, Hf, Q_new = self.compute_z_and_jacobians(particle, landmark, Q)
        dz = z[0:2].reshape(2, 1) - zp
        dz[1, 0] = pi_2_pi(dz[1, 0])

        try:
            invQ = np.linalg.inv(Q_new)
        except np.linalg.linalg.LinAlgError:
            raise ValueError("Q Not Invertable in compute weight")
        gaussian = math.exp(-0.5 * dz.T @ invQ @ dz)
        constant = math.sqrt(np.linalg.det(2.0 * math.pi * Q_new))
        w = gaussian / constant
        return w

    def compute_weight_map(self, particle, z: np.array, Q: np.array) -> float:
        """
        computes the importance weight for a single particle for a single landmark

        args:
        particle - Particle for which to compute weight for
        z - observation of landmark for the current time step
        Q - measurement covariance, as np array

        returns:
        w - the importance weight for this landmark

        """
        map_line_idx = int(z[2])
        z_hat, idx = particle.get_expected_map_observations(self.map_lines, z[1])
        if idx == -1:
            dz = 111 - z_hat
        #print_observations(z_hat)
        dz = z[0:2].reshape(2, 1) - z_hat
        dz[1, 0] = pi_2_pi(dz[1, 0])

        try:
            invQ = np.linalg.inv(Q)
        except np.linalg.linalg.LinAlgError:
            raise ValueError("Q Not Invertable in compute weight")
        gaussian = math.exp(-0.5 * dz.T @ invQ @ dz)
        constant = math.sqrt(np.linalg.det(2.0 * math.pi * Q))
        w = gaussian / constant
        return w

    def compute_z_and_jacobians(
        self, particle: Particle, landmark: Landmark, Q: np.array
    ) -> Tuple[np.array]:
        """
        computes empirical z, jacobian, and measurement covariance

        args:
        particle: current particle for which jacobian is being calculated
        landmark: current mu, sigma for landmark for this particle
        Q: measurement covariance as np array

        returns:
        z_hat: the expected location of the landmark from the measurement model
        Hv: Jacobian
        Hf: Measurment Jacobian
        Q_new: updated measurement covariance
        """
        dx = landmark.mu[0, 0] - particle.x()
        dy = landmark.mu[1, 0] - particle.y()
        r_squared = dx**2 + dy**2
        r = math.sqrt(r_squared)

        z_hat = np.array([[r], [pi_2_pi(math.atan2(dy, dx) - particle.orientation())]])
        Hv = np.array(
            [[-dx / r, -dy / r, 0.0], [dy / r_squared, -dx / r_squared, -1.0]]
        )
        Hf = np.array([[1, 0], [0, 1]])
        Q_new = Hf @ landmark.sigma @ Hf.T + Q

        return z_hat, Hv, Hf, Q_new

    def extend_path(self, d_angle: float) -> List[Particle]:
        """
        Moves all particles one time step forward, extending path posterior
        by sampling based on measurement model

        delta angle should be given in radians
        """
        particles = []
        for i in range(len(self.particles)):
            noise = self.generate_noise()
            new_particle = self.motion_model(self.ctx, self.particles[i], d_angle, noise)
            new_particle.set_landmarks(self.particles[i].landmarks)
            particles.append(new_particle)
        return particles

    def feature_extraction(self, z):
        if self.ctx["N_LANDMARKS"] == 0:
            return z, np.array([])
        pos = Particle.init_from_numpy(self.ctx, self.compute_MC_expected_position())
        z_landmark = []
        z_map = []
        for i in range(z.shape[1]):
            angle = z[1, i]
            r = z[0, i]
            wall_observation, _ = pos.get_expected_map_observations(
                self.map_lines, angle
            )
            dz = abs(wall_observation[0, 0] - r)
            if dz > self.ctx["LM_THRESH"]:
                z_landmark.append([r, angle, 0])
            else:
                z_map.append([r, angle, 0])
        return np.array(z_map).T, np.array(z_landmark).T

    def generate_noise(self):
        return (np.random.randn(1, 2) @ self.ctx["R"]).T

    def get_particles(self):
        return self.particles

    def landmark_extraction(self, z):
        z_lines = []
        if z.shape[1] > 1:
            left = z[:2, 0].reshape(2, 1)
            l_x, l_y = left[0] * math.cos(left[1]), left[0] * math.sin(left[1])
            right = z[:2, -1].reshape(2, 1)
            r_x, r_y = right[0] * math.cos(right[1]), right[0] * math.sin(right[1])
            right = np.array([r_x, r_y])
            left = np.array([l_x, l_y])
            line = ParametricLine(right, left - right)
            x3 = line.evaluate(0.5)
            ortho = line.get_orthogonal_slope()
            c = math.sqrt(self.ctx["LANDMARK_R"] ** 2 / np.sum(ortho**2))
            if ortho.reshape(1, 2).dot(x3)[0, 0] > 0:
                new_pos = x3 + c * ortho
            else:
                new_pos = x3 + c * -ortho
            new_r = math.sqrt(np.sum(new_pos**2))
            theta = pi_2_pi(math.atan2(new_pos[1, 0], new_pos[0, 0]))
            return np.array([[new_r], [theta], [0]])
        else:
            z = z[:, 0]
            theta = z[1]
            r = z[0] + self.ctx["LANDMARK_R"]
            #print('EXTRACT', r, theta)
            return np.array([[r], [theta], [0]])

    def normalize_weights(self):
        sumw = sum([p.importance_weight for p in self.particles])
        if sumw != 0:
            for i in range(len(self.particles)):
                self.particles[i].importance_weight /= sumw
        else:
            for i in range(len(self.particles)):
                self.particles[i].importance_weight = (
                    1.0 / self.ctx["N_PARTICLES"]
                )  # default weight
            return self.particles
        return self.particles

    def resampling(self):
        """
        resamples particles based on importance weights

        args:
        particles: list of particles to sample from

        returns:
        array of new particles
        """
        particles = self.normalize_weights()

        pw = []
        for i in range(len(particles)):
            pw.append(particles[i].importance_weight)

        pw = np.array(pw)

        wcum = np.cumsum(pw)
        base = (
            np.cumsum(pw * 0.0 + 1 / self.ctx["N_PARTICLES"])
            - 1 / self.ctx["N_PARTICLES"]
        )
        resampleid = base + np.random.rand(base.shape[0]) / self.ctx["N_PARTICLES"]

        inds = []
        ind = 0
        for ip in range(self.ctx["N_PARTICLES"]):
            while (ind < wcum.shape[0] - 1) and (resampleid[ip] > wcum[ind]):
                ind += 1
            inds.append(ind)

        resampled_particles = []
        for i in range(len(inds)):
            new_particle = self.particles[inds[i]].copy()
            new_particle.old_weight = self.particles[inds[i]].importance_weight
            #print("OLD", *self.particles[inds[i]].landmarks)
            #print("NEW", *new_particle.landmarks)
            resampled_particles.append(new_particle)

        return resampled_particles

    def run(self, d_angle, z, is_moving=True):
        #print("START", *self.particles[0].landmarks)
        if is_moving:
            self.particles = self.extend_path(d_angle)
        #print("POST MOVING", *self.particles[0].landmarks)

        if np.any(z):
            z_map, z_landmark = self.feature_extraction(z)
            #print("POST FEATURE EXTRACT", *self.particles[0].landmarks)

            n_feature_obs = 0
            if np.any(z_landmark):
                n_feature_obs = z_landmark.shape[1]
                z_landmark = self.landmark_extraction(z_landmark)
            #print("POST LANDMARK EXTRACT", *self.particles[0].landmarks)

            if z_map.shape[0] != 0:
                self.update_weights_map(z_map)
            #print("POST MAP UPDATE", *self.particles[0].landmarks)
            if z_landmark.shape[0] != 0:
                self.update_weights_landmarks(z_landmark, n_feature_obs)
            #print("POST LANDMARK UPDATE", *self.particles[0].landmarks)

        self.particles = self.resampling()

        #print("POST RESAMPLE", *self.particles[0].landmarks)

    def update_EKF(
        self, landmark: Landmark, dz: np.array, Q: np.array, Hf: np.array, n_feature_obs
    ):
        """
        Does a single EKF update for a single landmark, given
        difference between observed and expected observations
        and Jacobian for measurement model

        args:
        landmark: landmark to do EKF on
        dz: delta z = z_observed - z_hat
        Q: measurement covariance
        Hf: Jacobian of measurement noise

        returns:
        mu, sigma: updated belief for landmark
        """
        landmark_mu = landmark.mu
        landmark_sigma = landmark.sigma

        scaled_Q = max(4 - n_feature_obs, 1) * Q

        kalman_gain = landmark_sigma @ Hf.T @ np.linalg.inv(scaled_Q)
        mu = landmark_mu + kalman_gain @ dz
        sigma = (np.identity(2) - kalman_gain @ Hf) @ landmark_sigma
        landmark.mu = mu
        landmark.sigma = sigma
        return mu, sigma

    def update_landmark(
        self, particle: Particle, z: np.array, Q: np.array, n_feature_obs
    ) -> Particle:
        """
        Updates landmarks that were observed in z by doing
        an EKF update

        args:
        particle: particle that is being updated
        z: measurement for a particular landmark
        Q: measurement covariance

        returns:
        particle: updated particle
        """
        landmark_idx = int(z[2])
        landmark = particle.landmarks[landmark_idx]

        zp, Hv, Hf, Q = self.compute_z_and_jacobians(particle, landmark, Q)

        dz = z[0:2].reshape(2, 1) - zp
        dz[1, 0] = pi_2_pi(dz[1, 0])

        xf, Pf = self.update_EKF(landmark, dz, Q, Hf, n_feature_obs)

        return particle

    def update_weights_landmarks(self, z: np.array, n_feature_obs) -> List[Particle]:
        """
        Updates importance weights for each particle by on the
        the difference between the observed z and what the particle should see

        args:
        Updates the belief of the position of the landmarks for each particle based on
        new observations, doing an EKF update

        particles - list of current monte carlo particles
        z - list of observations for this time step

        returns:
        particles - updated particles
        """
        for iz in range(len(z[0, :])):  # for each observation
            landmark_idx = int(z[2, iz])  # get landmark idx

            for i in range(len(self.particles)):  # for each particle
                if (
                    abs(self.particles[i].landmarks[landmark_idx].mu[0]) <= 0.01
                ):  # if landmark does not exist yet
                    #print("ADDING NEW LANDMARK ************************************")
                    #print(landmark_idx, self.particles[i].landmarks[landmark_idx].mu[0])
                    self.particles[i] = self.add_new_landmark(
                        self.particles[i], z[:, iz], self.ctx["Q"]
                    )  # add landmark
                else:
                    w = self.compute_weight(
                        self.particles[i], z[:, iz], self.ctx["Q"]
                    )  # compute importance weight
                    self.particles[
                        i
                    ].importance_weight *= w  # update particle importance weight
                    self.particles[i] = self.update_landmark(
                        self.particles[i], z[:, iz], self.ctx["Q"], n_feature_obs
                    )  # update landmark posterior
        return self.particles

    def update_weights_map(self, z: np.array) -> List[Particle]:
        """
        Updates importance weights for each particle by on the
        the difference between the observed z and what the particle should see

        args:
        Updates the belief of the position of the landmarks for each particle based on
        new observations, doing an EKF update

        particles - list of current monte carlo particles
        z - list of observations for this time step

        returns:
        particles - updated particles
        """
        for iz in range(z.shape[1]):  # for each observation
            landmark_idx = int(z[2, iz])  # get landmark idx
            for i in range(len(self.particles)):  # for each particle
                w = self.compute_weight_map(
                    self.particles[i], z[:, iz], self.ctx["Q"]
                )  # compute importance weight
                self.particles[
                    i
                ].importance_weight *= w  # update particle importance weight
        return self.particles


def motion_model(
    ctx, particle: Particle, d_angle: float, noise: np.array = np.array([[0], [0]])
) -> Particle:
    """
    Motion model for robot, given delta angle
    Assumes constant speed
    delta angle should be given in radians
    """
    B = np.array(
        [
            [
                (ctx["DELTA"] + noise[0, 0])
                * math.sin(particle.orientation() + d_angle + noise[1, 0])
            ],
            [
                (ctx["DELTA"] + noise[0, 0])
                * math.cos(particle.orientation() + d_angle + noise[1, 0])
            ],
            [d_angle + noise[1, 0]],
        ]
    )
    sample_particle = particle.add(B)
    return sample_particle

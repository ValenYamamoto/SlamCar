from typing import List, Tuple
import math
import numpy as np
import yaml


class Landmark:
    def __init__(self):
        self.mu = np.zeros((2, 1))
        self.sigma = np.zeros((2, 2))


class Particle:
    """
    only weight should be mutable
    """

    @classmethod
    def init_from_numpy(cls, ctx: "SLAMContext", a):
        return cls(ctx, a[0, 0], a[1, 0], a[2, 0])

    def __init__(self, ctx, x=0.0, y=0.0, theta=0.0):
        self.importance_weight = 1.0 / ctx["N_PARTICLES"]
        self.old_weight = 0
        self.pos = np.array([[x], [y], [theta]])
        self.ctx = ctx

        self.landmarks = [Landmark() for i in range(ctx["N_LANDMARKS"])]

    def add(self, other) -> "Particle":
        if isinstance(other, np.ndarray):
            sample = Particle.init_from_numpy(self.ctx, self.pos + other)
            sample.fix_angle()
            return sample
        return NotImplemented

    def copy(self) -> "Particle":
        new_particle = Particle(self.ctx, self.x(), self.y(), self.orientation())
        new_particle.set_landmarks(self.landmarks)
        return new_particle

    def fix_angle(self) -> None:
        self.pos[2, 0] = pi_2_pi(self.orientation())

    def get_expected_map_observations(
        self, map_lines: List["ParametricLine"], angle: float
    ):
        """get z_hat"""
        theta = self.orientation()
        slope = np.array(
            [
                [self.ctx["RANGE"] * np.cos(angle + theta)],
                [self.ctx["RANGE"] * np.sin(angle + theta)],
            ]
        )
        line = ParametricLine(self.pos[:2, :], slope)
        z = []
        status, t, idx = get_intersection_with_map(map_lines, line)
        if status:
            return np.array([[line.r(t)], [angle]]), idx  # found intersection
        else:
            return np.array([[self.ctx["RANGE"]], [angle]]), -1  # no intersection

    def orientation(self):
        return self.pos[2, 0]

    def set_landmarks(self, landmarks):
        self.landmarks = landmarks

    def x(self):
        return self.pos[0, 0]

    def y(self):
        return self.pos[1, 0]

    def __str__(self):
        return f"{self.pos}"


class ParametricLine:
    def __init__(self, x, slope):
        self.x = x
        self.slope = slope
        if not np.any(self.slope):
            raise ValueError("Slope cannot be 0")

    def distance(self):
        return math.sqrt(slope[0] ** 2 + slope[1] ** 2)

    def evaluate(self, t):
        return self.x + t * self.slope

    def intersection(self, other):
        det = other.slope[0] * self.slope[1] - other.slope[1] * self.slope[0]
        if det == 0:
            return False, 0
        t = (
            self.slope[1] * (self.x[0] - other.x[0])
            - self.slope[0] * (self.x[1] - other.x[1])
        ) / det
        s = (
            -other.slope[1] * (self.x[0] - other.x[0])
            + other.slope[0] * (self.x[1] - other.x[1])
        ) / -det
        if 0 <= s <= 1 and 0 <= t <= 1:
            intersect = self.x + s * self.slope
            return True, s
        return False, 0

    def get_orthogonal_slope(self):
        s1, s2 = self.slope[0, 0], self.slope[1, 0]
        return np.array([[-s2], [s1]])

    def r(self, t):
        d = (t * self.slope) ** 2
        return math.sqrt(d.sum())


def LOG(s, is_printing=True):
    if is_printing:
        print(s)
    with open(LOGFILE, "a") as f:
        f.write(s + "\n")

def get_cartesian(z: np.array) -> np.array:
    r = z[0]
    theta = z[1]
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return np.array([[x], [y]])


def get_intersection_with_map(
    map_lines: List[ParametricLine], line: ParametricLine
) -> Tuple[bool, float, int]:
    min_distance = 1000
    ret = False, 0, -1
    for i, map_line in enumerate(map_lines):
        status, t = line.intersection(map_line)
        if status:
            dist = line.r(t)
            if dist < min_distance:
                min_distance = dist
                ret = True, t, i
    return ret

def calculate_mc_estimate(particles):
    x, y, theta = 0, 0, 0
    for particle in particles:
        x += particle.x()
        y += particle.y()
        theta += particle.orientation()
    x /= len(particles)
    y /= len(particles)
    theta /= len(particles)
    return [x, y, theta]

def calculate_importance_weight_mc(particles):
    x, y, theta = 0, 0, 0
    weight_total = 0
    for particle in particles:
        x += particle.x() * particle.old_weight
        y += particle.y() * particle.old_weight
        theta += particle.orientation() * particle.old_weight
        weight_total += particle.old_weight
    x /= weight_total
    y /= weight_total
    theta /= weight_total
    return [x, y, theta]

def pi_2_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi

def read_yaml_file(filename):
    with open(filename) as f:
        params_dict = yaml.safe_load(f)
    return params_dict

def print_particles(particles):
    for i, particle in enumerate(particles):
        print(
            f"Particle {i}: {particle.x():.2f} {particle.y():.2f} {particle.orientation():.2f} {particle.old_weight:2f}"
        )

    mc = calculate_mc_estimate(particles)
    iw = calculate_importance_weight_mc(particles)
    print(f"MC Est: {mc} IW Est: {iw}")
    

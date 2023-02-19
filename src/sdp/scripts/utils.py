from enum import Enum
from typing import List, Tuple
import math
import numpy as np
import yaml
import socket

from copy import deepcopy


isJetson = not (socket.gethostname().startswith("DESK") or socket.gethostname().startswith("LAP"))
if isJetson:
    import rospy
    from sdp.srv import *

MAX_ANGLE = 180 
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15

HOST = '127.0.0.1'
PORT = 65432

class Moves(Enum):
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4

class Landmark:
    def __init__(self):
        self.mu = np.zeros((2, 1))
        self.sigma = np.zeros((2, 2))

    def __str__(self):
        return f"({self.mu[0, 0]} {self.mu[1, 0]})"


class Particle:
    """
    only weight should be mutable
    """

    @classmethod
    def init_from_numpy(cls, ctx: "SLAMContext", a):
        return cls(ctx, a[0, 0], a[1, 0], a[2, 0])

    def __init__(self, ctx, x=0.0, y=0.0, theta=0.0):
        self.importance_weight = 1.0 / ctx["N_PARTICLES"]
        self.old_weight = 1.0 / ctx["N_PARTICLES"]
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
                [self.ctx["RANGE"] * np.sin(angle + theta)],
                [self.ctx["RANGE"] * np.cos(angle + theta)],
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
        self.landmarks = deepcopy(landmarks)

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

def calculate_mc_landmark(ctx, particles):
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
    if "ANGLES" in params_dict:
        for i in range(len(params_dict["ANGLES"])):
            params_dict["ANGLES"][i] = np.deg2rad(params_dict["ANGLES"][i])
    if "MOVES" in params_dict:
        for i, v in enumerate(params_dict["MOVES"]):
            if v == "FORWARD":
                params_dict["MOVES"][i] = Moves.FORWARD
            elif v == "BACKWARD":
                params_dict["MOVES"][i] = Moves.BACKWARD
            elif v == "LEFT":
                params_dict["MOVES"][i] = Moves.LEFT
            elif v == "RIGHT":
                params_dict["MOVES"][i] = Moves.RIGHT
    if 'theta' in params_dict:
        params_dict['theta'] = np.deg2rad(params_dict['theta'])
    return params_dict

def generate_wall_lines(ctx):
    #assert "WALLS_X" in ctx and "WALLS_Y" in ctx
    walls = []
    for i in range(len(ctx["WALLS_X"])-1):
        start = np.array([[ctx["WALLS_X"][i]], [ctx["WALLS_Y"][i]]])
        slope = np.array([[ctx["WALLS_X"][i+1]], [ctx["WALLS_Y"][i+1]]]) - start

        walls.append(ParametricLine(start, slope))
    return walls

def generate_landmark_lines(ctx):
    walls = []
    for i in range(len(ctx["LANDMARK_X"])):
        print("HI1")
        bot_x = ctx["LANDMARK_X"][i] - ctx["LANDMARK_R"]
        bot_y = ctx["LANDMARK_Y"][i] - ctx["LANDMARK_R"]
        top_x = ctx["LANDMARK_X"][i] + ctx["LANDMARK_R"]
        top_y = ctx["LANDMARK_Y"][i] + ctx["LANDMARK_R"]
        print("HI")
        r = ctx["LANDMARK_R"] * 2

        start = np.array([[bot_x], [bot_y]])
        slope = np.array([[r], [0]])
        walls.append(ParametricLine(start, slope))

        start = np.array([[bot_x], [bot_y]])
        slope = np.array([[0], [r]])
        walls.append(ParametricLine(start, slope))

        start = np.array([[bot_x], [top_y]])
        slope = np.array([[r], [0]])
        walls.append(ParametricLine(start, slope))

        start = np.array([[top_x], [bot_y]])
        slope = np.array([[0], [r]])
        walls.append(ParametricLine(start, slope))
    return walls

def generate_spread_particles(ctx):
    initial_orientation = ctx["theta"]
    if len(ctx["WALLS_X"]) == 2:
        x_low, x_high = 0, np.max(ctx["WALLS_X"])-1
        x = np.linspace(x_low, x_high, ctx["N_PARTICLES"])
        y = ctx['y']
        particles = []
        for i in range(len(x)):
            particles.append(Particle(ctx, x[i], y, initial_orientation))
        return particles
    x_size = ctx["N_PARTICLES"] // 5
    y_size = ctx["N_PARTICLES"] // x_size
    x_low, x_high = np.min(ctx["WALLS_X"])+1, np.max(ctx["WALLS_X"])-1
    y_low, y_high = np.min(ctx["WALLS_Y"])+1, np.max(ctx["WALLS_Y"])-1
    x = np.linspace(x_low, x_high, x_size)
    y = np.linspace(y_low, y_high, y_size)
    particles = []
    for xi in x:
        for yi in y:
            particles.append(Particle(ctx, xi, yi, initial_orientation))
    return particles

def print_particles(particles):
    for i, particle in enumerate(particles):
        print(
            f"Particle {i}: {particle.x():.2f} {particle.y():.2f} {particle.orientation():.2f} {particle.old_weight:2f} \
                    {particle.landmarks[i]}"
        )

    mc = calculate_mc_estimate(particles)
    iw = calculate_importance_weight_mc(particles)
    print(f"MC Est: {mc} IW Est: {iw}")
    

def log_particles(particles, socket=False):
    global isJetson
    mc = calculate_mc_estimate(particles)
    iw = calculate_importance_weight_mc(particles)
    if isJetson:
        for i, particle in enumerate(particles):
            rospy.loginfo(
                f"Particle {i}: {particle.x():.2f} {particle.y():.2f} {particle.orientation():.2f} {particle.old_weight:2f}"
            )
        rospy.loginfo(f"MCEst: {mc}")
        rospy.loginfo(f"IWEst: {iw}")
    else:
        for i, particle in enumerate(particles):
            print(
                f"Particle {i}: {particle.x():.2f} {particle.y():.2f} {particle.orientation():.2f} {particle.old_weight:2f} {particle.landmarks[0]}"
            )
        print(f"MCEst: {mc}")
        print(f"IWEst: {iw}")
    if socket:
        socket.send(create_particle_string(particles))

def create_particle_string(particles):
    s = ''
    for particle in particles:
        s += f"{particle.x():6.2f} {particle.y():6.2f} {particle.old_weight:6.2f}"
        for landmark in particle.landmarks:
            s += f" {landmark.mu[0,0]:6.2f} {landmark.mu[1,0]:6.2f}"
        s += "\r\n"

    return s
        
def scale_servo_angle(angle):
    return (angle + math.pi) / (2 * math.pi) * 180


def turn_to_servo_angle(phi):
    """ phi is in radians"""
    return round(180.49955056 * phi + 101.42095328055964)
    

def create_observations(ctx, sensor_data):
    distance = []
    angles = []
    landmark = []
    for data, angle in zip(sensor_data, ctx["ANGLES"]):
        if data == -1:
            continue
        else:
            distance.append(data+5.08)
            angles.append(-angle)
            landmark.append(0) # hardcode for now
    return np.array([distance, angles, landmark])

def servo_control_client(a, ch):
    global isJetson
    if not isJetson:
        return 0
    rospy.wait_for_service('servo_control_srv')
    try:
        servo_control_req = rospy.ServiceProxy('servo_control_srv', ServoData)
        # servo_control_req = f1tenth_simulator.srv.
        servo_control_resp = servo_control_req(angle=a)
        return servo_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def motor_control_client(s, rev):
    global isJetson
    if not isJetson:
        return 0
    rospy.wait_for_service('motor_control_srv')
    try:
        motor_control_req = rospy.ServiceProxy('motor_control_srv', MotorData)
        motor_control_resp = motor_control_req(speed_percent=s, is_reverse=rev)
        return motor_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def move_to_angle(move):
    if move == Moves.FORWARD:
        return 0
    elif move == Moves.BACKWARD:
        return 180 # TODO fix
    elif move == Moves.LEFT:
        return np.deg2rad(-20)
    elif move == Moves.RIGHT:
        return np.deg2rad(20)

def move_jetson(motor_speed, turn_angle):
    err = motor_control_client(motor_speed, 0)
    err = servo_control_client(turn_to_servo_angle(-turn_angle), SERVO_CHANNEL)


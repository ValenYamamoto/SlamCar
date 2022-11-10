#!/usr/bin/python3
from __future__ import print_function

import sys
import rospy
from sdp.srv import *

import board
import adafruit_vl53l4cd

import argparse
import numpy as np

from slam import FastSLAM, SLAMContext, motion_model
from utils import Particle, ParametricLine, read_yaml_file, log_particles

MAX_ANGLE = 180 
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15


def servo_control_client(a, ch):
    rospy.wait_for_service('servo_control_srv')
    try:
        servo_control_req = rospy.ServiceProxy('servo_control_srv', ServoData)
        # servo_control_req = f1tenth_simulator.srv.
        servo_control_resp = servo_control_req(angle=a)
        return servo_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def motor_control_client(s, rev):
    rospy.wait_for_service('motor_control_srv')
    try:
        motor_control_req = rospy.ServiceProxy('motor_control_srv', MotorData)
        motor_control_resp = motor_control_req(speed_percent=s, is_reverse=rev)
        return motor_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def noise_function(self):
    a = (np.random.randn(1, 2) @ self.ctx["R"]).T
    a[1,0] = 0
    return a

def wall_generator(wall_distance):
    return [ParametricLine(np.array([[wall_distance],[-1]]), np.array([[0], [1]]))]

def get_observation(wall_distance):
    obs = wall_distance
    while True:
        yield np.array([[obs], [0], [0]])
        obs -= 5

def generate_spread_particles(ctx, mini, maxi):
    x = np.linspace(mini, maxi, ctx["N_PARTICLES"])
    particles = []
    for pos in x:
        particles.append(Particle(ctx, pos, 0, 0))
    return particles

WALL_DISTANCE = 100

def get_range(vl53):
    vl53.start_ranging()
    total = 0
    for i in range(3):
        while not vl53.data_ready:
            pass

        vl53.clear_interrupt()
        total += vl53.distance
    vl53.stop_ranging()
    return total / 3

if __name__ == "__main__":
    rospy.init_node("WallTest")
    i2c = board.I2C()
    vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)

    #parser = argparse.ArgumentParser()
    #parser.add_argument("-y", "--yaml", default="")
    #args = parser.parse_args()

    x = 0
    y = 0
    theta = 0

    if True: 
        params_dict = read_yaml_file("/home/sdp10/catkin_ws/src/sdp/scripts/params.yaml")
        ctx = SLAMContext.init_from_dict(params_dict)
        if 'x' in params_dict:
            x = params_dict['x']
        if 'y' in params_dict:
            y = params_dict['y']
        if 'theta' in params_dict:
            theta = params_dict['theta']
        if 'WALL_DISTANCE' in params_dict:
            WALL_DISTANCE = params_dict['WALL_DISTANCE']
    else:
        ctx = SLAMContext()

    particles = generate_spread_particles(ctx, 0, WALL_DISTANCE)
    map_lines = wall_generator(WALL_DISTANCE)

    fastSlam = FastSLAM(ctx, x, y, theta, motion_model, map_lines)
    fastSlam.particles = particles
    setattr(FastSLAM, 'generate_noise', noise_function)

    obs_generator = get_observation(WALL_DISTANCE - x)
    log_particles(fastSlam.particles)
    rospy.loginfo("READY")

    z = get_range(vl53) 
    r = rospy.Rate(2) # in hz
    input("press to begin")
    while True:
        z = get_range(vl53) 
        rospy.loginfo(f"Z {z}")
        z = np.array([[z],[0], [0]])
        fastSlam.run(0, z)
        if z[0,0] < 10 or abs(z[0,0] - 10) < 1:
            break
        log_particles(fastSlam.particles)
        err = motor_control_client(10, 0)
        rospy.loginfo("moving motors")
        r.sleep()
    err = motor_control_client(0, 0)
    rospy.loginfo("Stopping motors")
    log_particles(fastSlam.particles)

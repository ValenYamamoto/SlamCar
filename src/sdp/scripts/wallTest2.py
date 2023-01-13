#!/usr/bin/python3
from __future__ import print_function

import sys
import socket

import numpy as np

from slam import FastSLAM, SLAMContext, motion_model
from utils import Particle, ParametricLine, read_yaml_file, log_particles, get_intersection_with_map
#from drawer import Drawer

isJetson = not socket.gethostname().startswith("DESK")

if isJetson:
    import rospy
    from sdp.srv import *

    import board
    import adafruit_vl53l4cd
else:
    import argparse

MAX_ANGLE = 180 
MIN_ANGLE = 0
ANGLE_STEP = 18

MAX_SPD = 100
MIN_SPD = 0
SPD_STEP = 5

SERVO_CHANNEL = 15

WALL_DISTANCE = 100


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

def noise_function(self):
    a = (np.random.randn(1, 2) @ self.ctx["R"]).T
    a[1,0] = 0
    return a

def wall_generator(wall_distance):
    return [ParametricLine(np.array([[wall_distance],[-10.31875]]), np.array([[0], [20.31875]]))]

def get_observation(ctx, wall_distance, map_lines, drawer):
    obs = wall_distance
    slopes = [
        np.array(
            [
                [ctx["RANGE"] * np.cos(angle)],
                [ctx["RANGE"] * np.sin(angle)],
            ]
        )
        for angle in ctx["ANGLES"]
    ]
    while True:
        lines = [ParametricLine(np.array([[wall_distance], [0]]), slope) for slope in slopes]
        #drawer.draw_lines(lines, 'g')
        line_intersects = []
        z_map = []
        print("WALL DIST:", wall_distance)
        for angle, line in zip(ctx["ANGLES"], lines):
            status, t, idx = get_intersection_with_map(map_lines, line)
            if status:
                z_map.append([line.r(t), angle, 0])  # hardcode 0 for now
        yield np.array(z_map).T
        wall_distance += ctx["DELTA"]

def generate_spread_particles(ctx, mini, maxi):
    x = np.linspace(mini, maxi, ctx["N_PARTICLES"])
    particles = []
    for pos in x:
        particles.append(Particle(ctx, pos, 0, 0))
    return particles


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

def loginfo(s):
    global isJetson
    if isJetson:
        rospy.loginfo(s)
    else:
        print(s)

if __name__ == "__main__":
    if isJetson:
        rospy.init_node("WallTest")
        i2c = board.I2C()
        vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)
    else:
        parser = argparse.ArgumentParser()
        parser.add_argument("-y", "--yaml", default="")
        args = parser.parse_args()

    x = 0
    y = 0
    theta = 0

    if isJetson: 
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
    elif args.yaml:
        params_dict = read_yaml_file(args.yaml)
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
    print("MAP", map_lines)

    fastSlam = FastSLAM(ctx, x, y, theta, motion_model, map_lines)
    fastSlam.particles = particles
    #drawer = Drawer(ctx, fastSlam, [0, 101], [-5, 5], map_lines, [])
    #drawer.draw()
    #drawer.save_image('status_out.png')
    setattr(FastSLAM, 'generate_noise', noise_function)

    obs_generator = get_observation(ctx, x, map_lines, None)
    log_particles(fastSlam.particles)
    loginfo("READY")

    if isJetson:
        r = rospy.Rate(1) # in hz
    input("press to begin")
    while True:
        z = next(obs_generator)
        loginfo(f"Z {z}")
        fastSlam.run(0, z)
        if z.size > 0 and (np.any(z[0,:] < 10) or np.any(abs(z[0,:] - 10) < 1)):
            break
        log_particles(fastSlam.particles)
        #drawer.draw()
        #drawer.save_image('status_out.png')
        err = motor_control_client(10, 0)
        loginfo("moving motors")
        if isJetson:
            r.sleep()
        else:
            input("press to continue")
    #drawer.draw()
    #drawer.save_image('status_out.png')
    err = motor_control_client(0, 0)
    loginfo("Stopping motors")
    log_particles(fastSlam.particles)

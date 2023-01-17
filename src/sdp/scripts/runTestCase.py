#!/usr/bin/python3
from __future__ import print_function

import sys
import socket
import time

import numpy as np

from slam import FastSLAM, SLAMContext, motion_model
from utils import (
    Particle, 
    ParametricLine, 
    read_yaml_file, 
    log_particles, 
    get_intersection_with_map,
    servo_control_client,
    motor_control_client,
    create_observations
)

from dashboard.dashboard_socket import DashboardSocket
    

isJetson = not socket.gethostname().startswith("DESK")

if isJetson:
    import rospy
    from sdp.srv import *

    import board
    from sdp.msg import ToFData
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

sensor_data = None

# for the dashboard socket
HOST = '127.0.0.1'
PORT = 65432

def tof_callback(data):
    global sensor_data
    sensor_data = data.sensor_data

def wall_generator(wall_distance):
    return [ParametricLine(np.array([[wall_distance],[-32]]), np.array([[0], [64]]))]

def get_observation(ctx, wall_distance, map_lines):
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
        line_intersects = []
        z_map = []
        print("WALL DIST:", wall_distance)
        for angle, line in zip(ctx["ANGLES"], lines):
            status, t, idx = get_intersection_with_map(map_lines, line)
            if status:
                z_map.append([line.r(t), angle, 0])  # hardcode 0 for now
        yield np.array(z_map).T
        wall_distance += ctx["DELTA"]

def update_position(move):
    pass

def generate_spread_particles(ctx, mini, maxi):
    x = np.linspace(mini, maxi, ctx["N_PARTICLES"])
    particles = []
    for pos in x:
        particles.append(Particle(ctx, pos, 0, 0))
    return particles


def loginfo(s):
    global isJetson
    if isJetson:
        rospy.loginfo(s)
    else:
        print(s)

if __name__ == "__main__":
    if isJetson:
        rospy.init_node("WallTest")
        rospy.Subscriber('/tof_data', ToFData, tof_callback)
        i2c = board.I2C()
    else:
        parser = argparse.ArgumentParser()
        parser.add_argument("-y", "--yaml", default="")
        args = parser.parse_args()

    socket = DashboardSocket(True, HOST, PORT)
    try:
        socket.connect()
    except:
        print("Socket did not connect") 
        exit(1)
    try:

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

        # setup particles
        particles = generate_spread_particles(ctx, 0, WALL_DISTANCE)

        # setup wall
        map_lines = wall_generator(WALL_DISTANCE)
        print("MAP", map_lines)

        # setup FastSLAM instance for localization
        fastSlam = FastSLAM(ctx, x, y, theta, motion_model, map_lines)
        fastSlam.particles = particles

        # get observation generator for simulation
        obs_generator = get_observation(ctx, x, map_lines)

        # log initial state
        log_particles(fastSlam.particles, socket=socket)
        loginfo("READY")

        if isJetson:
            r = rospy.Rate(ctx["RATE"]) # in hz
            print("Waiting for sensor data")
            while sensor_data is None:
                time.sleep(0.5)
                continue 
            print("Sensor data ready")

        input("press to begin")

        for move in ctx["MOVES"]:
            if not isJetson:
                z = next(obs_generator)
            else:
                z = create_observations(ctx, sensor_data)
            loginfo(f"Z {z}")
            fastSlam.run(0, z)
            if z.size > 0 and (np.any(z[0,:] < 10) or np.any(abs(z[0,:] - 10) < 1)):
                break
            log_particles(fastSlam.particles, socket=socket)
            err = motor_control_client(10, 0)
            loginfo("moving motors")
            if isJetson:
                input("press to continue")
                #r.sleep()
            else:
                input("press to continue")
        err = motor_control_client(0, 0)
        loginfo("Stopping motors")
        log_particles(fastSlam.particles, socket=socket)
    finally:
        socket.close()

#!/usr/bin/python3
from __future__ import print_function

import sys
import socket
import time

import numpy as np

from slam import FastSLAM, SLAMContext, motion_model
from utils import (
    Moves,
    Particle, 
    ParametricLine, 
    read_yaml_file, 
    log_particles, 
    get_intersection_with_map,
    servo_control_client,
    motor_control_client,
    create_observations,
    generate_wall_lines,
    generate_spread_particles,
    move_to_angle
)

from dashboard.dashboard_socket import DashboardSocket
    

isJetson = not (socket.gethostname().startswith("DESK") or socket.gethostname().startswith("LAP"))

if isJetson:
    import rospy
    from sdp.srv import *

    import board
    from sdp.msg import ToFData
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
    while True:
        slopes = [
            np.array(
                [
                    [ctx["RANGE"] * np.cos(angle+ctx['theta'])],
                    [ctx["RANGE"] * np.sin(angle+ctx['theta'])],
                ]
            )
            for angle in ctx["ANGLES"]
        ]
        lines = [ParametricLine(np.array([[ctx['x']], [ctx['y']]]), slope) for slope in slopes]
        line_intersects = []
        z_map = []
        for angle, line in zip(ctx["ANGLES"], lines):
            status, t, idx = get_intersection_with_map(map_lines, line)
            if status:
                z_map.append([line.r(t), angle, 0])  # hardcode 0 for now
        yield np.array(z_map).T

def update_position(ctx, move):
    position = Particle(ctx, ctx['x'], ctx['y'], ctx['theta'])
    if move == Moves.FORWARD:
        particle = motion_model(ctx, position, 0)
    elif move == Moves.BACKWARD:
        particle = motion_model(ctx, position, 180) # TODO fix
    elif move == Moves.LEFT:
        particle = motion_model(ctx, position, np.deg2rad(20/ctx["RATE"]))
    elif move == Moves.RIGHT:
        particle = motion_model(ctx, position, np.deg2rad(-20/ctx["RATE"]))
    ctx['x'] = particle.x()
    ctx['y'] = particle.y()
    ctx['theta'] = particle.orientation()

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
        parser = argparse.ArgumentParser()
        parser.add_argument("-y", "--yaml", default="")
        parser.add_argument("-s", "--simulation", action='store_true')
        args = parser.parse_args()

    socket = DashboardSocket(True, HOST, PORT)
    try:
        socket.connect()
    except:
        print("Socket did not connect") 
        socket=False
    try:

        x = 0
        y = 0
        theta = 0

        err = servo_control_client(100, SERVO_CHANNEL)

        if isJetson: 
            #params_dict = read_yaml_file("/home/sdp10/catkin_ws/src/sdp/scripts/params.yaml")
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

        # setup wall
        map_lines = generate_wall_lines(ctx)

        # setup FastSLAM instance
        fastSlam = FastSLAM(ctx, x, y, theta, motion_model, map_lines)
        # setup particles
        if params_dict["PARTICLE_STATE"]=='localize':
            particles = generate_spread_particles(ctx)
            fastSlam.particles = particles

        # get observation generator for simulation
        obs_generator = get_observation(ctx, x, map_lines)

        # log initial state
        print(f"POSITION x: {ctx['x']} y: {ctx['y']} theta: {ctx['theta']}")
        log_particles(fastSlam.particles, socket=socket)
        print()

        if isJetson:
            r = rospy.Rate(ctx["RATE"]) # in hz
            print("Waiting for sensor data")
            while sensor_data is None:
                time.sleep(0.5)
                continue 
            print("Sensor data ready")

        if not args.simulation:
            input("press to begin")
            start = time.perf_counter_ns()

        #for move in ctx["MOVES"]:
        move = ctx["MOVES"][0]
        i = 1
        while i < len(ctx["MOVES"]):
            print("MOVE:", move)
            if not isJetson:
                update_position(ctx, move)
                print(f"POSITION x: {ctx['x']} y: {ctx['y']} theta: {ctx['theta']}")
                z = next(obs_generator)
            else:
                z = create_observations(ctx, sensor_data)
            loginfo(f"Z {repr(z)}")
            i+=1
            fastSlam.run(move_to_angle(move)/ctx["RATE"], z)
            if isJetson:
                err = motor_control_client(20, 0)
            """
            if z.size > 0 and (np.any(z[0,:] < 10) or np.any(abs(z[0,:] - 10) < 1)):
                break
            """
            log_particles(fastSlam.particles, socket=socket)
            if isJetson:
                #input("press to continue")
                r.sleep()
            else:
                if not args.simulation:
                    input("press to continue")
            print()
        err = motor_control_client(0, 0)
        if not args.simulation and isJetson:
            end = time.perf_counter_ns()
            elapsed = (end - start) / 1e9
            loginfo(f"elapsed: {elapsed}")
        loginfo("Stopping motors")
        # log_particles(fastSlam.particles, socket=socket)
    except Exception as e:
        print(e)
    finally:
        if socket is False:
            pass
        else:
            socket.close()

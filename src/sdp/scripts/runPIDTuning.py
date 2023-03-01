#!/usr/bin/python3
from __future__ import print_function

import sys
import socket
import time
import traceback

import numpy as np

from auto import FSM, State, auto_move_to_angle, StraightPID
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
    generate_auto_walls,
    generate_landmark_lines,
    generate_spread_particles,
    move_to_angle,
    move_jetson
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

def generate_walls():
    w1 = ParametricLine(np.array([[0],[0]]), np.array([[0], [300]]))
    w2 = ParametricLine(np.array([[70],[0]]), np.array([[0], [300]]))
    return [w1, w2]

def get_observation(ctx, wall_distance, map_lines, landmark_lines):
    while True:
        slopes = [
            np.array(
                [
                    [ctx["RANGE"] * np.sin(angle+ctx['theta'])],
                    [ctx["RANGE"] * np.cos(angle+ctx['theta'])],
                ]
            )
            for angle in ctx["ANGLES"]
        ]
        lines = [ParametricLine(np.array([[ctx['x']], [ctx['y']]]), slope) for slope in slopes]
        line_intersects = []
        z_map = []
        for angle, line in zip(ctx["ANGLES"], lines):
            status, t, idx = get_intersection_with_map(map_lines, line)
            current = 200 # a large number
            if status and line.r(t) < current:
                current = line.r(t)
            status, t, idx = get_intersection_with_map(landmark_lines, line)
            if status and line.r(t) < current:
                current = line.r(t)

            if current == 200:
                continue
            z_map.append([current, angle, 0])  # hardcode 0 for now
            
        yield np.array(z_map).T

def update_position(ctx, move, angle):
    position = Particle(ctx, ctx['x'], ctx['y'], ctx['theta'])
    if move == Moves.FORWARD:
        particle = motion_model(ctx, position, angle)
    elif move == Moves.BACKWARD:
        particle = motion_model(ctx, position, 180) # TODO fix
    elif move == Moves.LEFT:
        particle = motion_model(ctx, position, np.deg2rad(-35/ctx["RATE"]))
    elif move == Moves.RIGHT:
        particle = motion_model(ctx, position, np.deg2rad(35/ctx["RATE"]))
    ctx['x'] = particle.x()
    ctx['y'] = particle.y()
    ctx['theta'] = particle.orientation()

def loginfo(s):
    global isJetson
    if isJetson:
        rospy.loginfo(s)
    else:
        print(s)

def print_observations(z):
    if not np.any(z):
        return
    for i in range(z.shape[1]):
        print(f"Angle: {z[1, i]} Dist: {z[0,i]}")

if __name__ == "__main__":
    if isJetson:
        rospy.init_node("AutoTest")
        rospy.Subscriber('/tof_data', ToFData, tof_callback)
        i2c = board.I2C()
    parser = argparse.ArgumentParser()
    parser.add_argument("-y", "--yaml", default="")
    parser.add_argument("-s", "--simulation", action='store_true')
    parser.add_argument("-t", "--track", action='store_true')
    parser.add_argument("-p", "--p", type=float, default=0)
    parser.add_argument("-i", "--i", type=float, default=0)
    parser.add_argument("-d", "--d", type=float, default=0)
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
        map_lines = generate_walls()
        landmark_lines = generate_landmark_lines(ctx)

        # setup FastSLAM instance
        fastSlam = FastSLAM(ctx, x, y, theta, motion_model, map_lines)

        straightPID = StraightPID(args.p, args.i, args.d, target=35)

        # get observation generator for simulation
        obs_generator = get_observation(ctx, x, map_lines, landmark_lines)

        # setup initial state
        pos = fastSlam.compute_MC_expected_position()
        #move = fsm.actions(state)
        #print("STATE:", state)
        #print("MOVE:", move)

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

        if isJetson:
            i = 0
            while i==0:
                move_jetson(20, auto_move_to_angle(move))
                i+= 1
                r.sleep()

        pid_result = 0
        i = 1
        # Run for a set number of cycles in case something goes wrong. 200/5 = ~40 secs
        while i < 30:
            if not isJetson:
                update_position(ctx, Moves.FORWARD, pid_result)
                print(f"POSITION x: {ctx['x']} y: {ctx['y']} theta: {ctx['theta']}")
                z = next(obs_generator)
            else:
                z = create_observations(ctx, sensor_data)
            #loginfo(f"Z {repr(z)}")
            print_observations(z)
            i+=1
            fastSlam.run(pid_result, z)

            # compute current position and get next move from FSM
            pos = fastSlam.compute_MC_expected_position()
            pid_result = straightPID.next(pos[0,0])

            if isJetson:
                move_jetson(20, auto_move_to_angle(pid_result))
                #move_jetson(20, auto_move_to_angle(move))

                # Exit condition for jetson
                if z.size > 0 and (sensor_data[2] < 11 or sensor_data[3] < 11): 
                    break
            else:
                # Exit condition for simulation
                exiting = False
                """
                if np.any(z):
                    for j in range(z.shape[1]):
                        if abs(abs(z[1,j]) - np.deg2rad(18)) < 0.01:
                            if z[0,j] < 11:
                                exiting = True
                                break
                if exiting:
                    break
                """

            log_particles(fastSlam.particles, socket=socket)
            print(f"PID: {pid_result}, x: {35 - pos[0,0]}")
            if isJetson:
                r.sleep()
            else:
                if not args.simulation:
                    input("press to continue")
            print()
        move_jetson(0, 0)
        if not args.simulation and isJetson:
            end = time.perf_counter_ns()
            elapsed = (end - start) / 1e9
            loginfo(f"elapsed: {elapsed}")
        loginfo("Stopping motors")
        # log_particles(fastSlam.particles, socket=socket)
    except Exception as e:
        print(e)
        traceback.print_exc()
    finally:
        move_jetson(0, 0)
        if socket is False:
            pass
        else:
            socket.close()

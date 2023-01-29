import sys
import re
import pandas as pd

from numpy import array


def parse_file(f):
    step = 0

    next(f)
    next(f)
    position_line = next(f)
    position = parse_position(position_line)

    initial_est = parse_particles(f)

    s = to_string(step, position, initial_est, "NONE", [])
    step += 1
    next(f)
    
    while True:
        move_line = next(f)
        if move_line.startswith("MOVE"):
            move = parse_move(move_line)

            position_line = next(f)
            position = parse_position(position_line)
            
            observations = parse_obs_array(f)

            est = parse_particles(f)

            s += to_string(step, position, est, move, observations)
            next(f)
            step += 1
        else:
            break
    return s

def to_string(step, position, est, move, obs):
    s = f'{step},"[{position[0]:.2f},{position[1]:.2f},{position[2]:.2f}]",'
    s += '"['
    for o in obs:
        s += f"({o[0]:.2f}, {o[1]:.2f}) "
    s += ']",'
    mc = est[0]
    iw = est[1]
    s += f'{move},"[{mc[0]:.2f},{mc[1]:.2f},{mc[2]:.2f}]","[{iw[0]:.2f},{iw[1]:.2f},{iw[2]:.2f}]",\n'
    return s


def parse_position(line):
    result = re.search("POSITION x: ([+-]?([0-9]*[.])?[0-9]+) y: ([+-]?([0-9]*[.])?[0-9]+) theta: ([+-]?([0-9]*[.])?[0-9]+)", line)
    x = float(result.group(1))
    y = float(result.group(3))
    theta = float(result.group(5))
    return x, y, theta

def parse_move(line):
    result = re.search("MOVE: Moves\.(.*)", line)
    move = result.group(1)
    return move

def parse_particles(f):
    iw, mc =0, 0
    for line in f:
        result = re.search("^([MI][CW])Est: (.*)", line.strip())
        if result:
            if result.group(1) == "MC":
                mc = eval(result.group(2))
            if result.group(1) == "IW":
                iw = eval(result.group(2))
                return iw, mc
        elif not line.startswith("Particle"):
            return 0, 0

def parse_obs_array(f):
    result = ""
    for line in f:
        if line.strip().endswith("])"):
            result += line
            break
        elif line.startswith("Z"):
            result += line[1:]
        else:
            result += line
    result = eval(result)
    processed = []
    for i in range(result.shape[1]):
        processed.append((result[0,i], result[1,i]))
    return processed


if __name__ == "__main__":
    with open(sys.argv[1]) as f:
        s = parse_file(f)
    print('Time Step, Actual Position,"Observations (distance, angle)",Move,Monte Carlo Est,Importance Weight Est,')
    print(s)


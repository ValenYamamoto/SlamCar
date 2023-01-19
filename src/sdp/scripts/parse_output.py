import sys
import re
import pandas as pd

from numpy import array


def parse_file(f):
    x, y, theta, mc, iw, obs, steps, moves = [], [], [], [], [], [], [], []
    step = 0
    steps.append(step)
    step += 1

    next(f)
    next(f)
    position_line = next(f)
    position = parse_position(position_line)

    x.append(position[0])
    y.append(position[1])
    theta.append(position[2])

    initial_est = parse_particles(f)
    mc.append(initial_est[0])
    iw.append(initial_est[1])

    moves.append("NONE")
    obs.append([])
    next(f)
    
    while True:
        move_line = next(f)
        if move_line.startswith("MOVE"):
            steps.append(step)
            step += 1
            move = parse_move(move_line)
            moves.append(move)

            position_line = next(f)
            position = parse_position(position_line)
            x.append(position[0])
            y.append(position[1])
            theta.append(position[2])
            
            observations = parse_obs_array(f)
            obs.append(observations)

            est = parse_particles(f)
            mc.append(est[0])
            iw.append(est[1])

            next(f)
        else:
            break
    df = pd.DataFrame(data={
        "step": steps,
        "x": x,
        "y": y,
        "theta": theta,
        "move": moves,
        "mc": mc,
        "iw": iw,
        "observations": obs,
    })
    return df



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
        df = parse_file(f)
    print(df.to_csv(index=False))


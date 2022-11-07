import argparse
import numpy as np

from slam import FastSLAM, SLAMContext, motion_model
from utils import Particle, ParametricLine, read_yaml_file, print_particles

# for 1-D, no noise for orientation
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-y", "--yaml", default="")
    args = parser.parse_args()

    x = 0
    y = 0
    theta = 0

    if args.yaml: 
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

    fastSlam = FastSLAM(ctx, x, y, theta, motion_model, map_lines)
    fastSlam.particles = particles
    setattr(FastSLAM, 'generate_noise', noise_function)

    obs_generator = get_observation(WALL_DISTANCE - x)
    print_particles(fastSlam.particles)

    z = next(obs_generator)
    z = next(obs_generator)
    while True:
        print("Z", z[0,0])
        fastSlam.run(0, z)
        if z[0,0] < 20 or abs(z[0,0] - 20) < 1:
            break
        z = next(obs_generator) 
        print_particles(fastSlam.particles)
        input()
    print_particles(fastSlam.particles)

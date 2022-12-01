import cv2
import math
import matplotlib.pyplot as plt
import numpy as np

from scipy.linalg import sqrtm
from matplotlib import cm
from matplotlib.patches import Patch, Rectangle
from matplotlib.lines import Line2D


from utils import Particle, ParametricLine, get_intersection_with_map
from slam import FastSLAM


class Drawer:
    def __init__(self, ctx, fastSLAM, xrange, yrange, map_lines, landmark_lines):
        self.map_lines = map_lines
        self.landmark_lines = landmark_lines
        self.ctx = ctx
        self.fastSlam = fastSLAM
        self.xrange = xrange
        self.yrange = yrange
        self.pos = None

    def draw(self):
        self.draw_map()
        self.draw_particles()
        # self.draw_true_position()
        plt.xlim(self.xrange)
        plt.ylim(self.yrange)

    def draw_gaussians(self, mean, covar, colors=None):
        """Arguments: list of means (length-2 numpy arrays) and covariances (2x2 arrays)"""
        from scipy.linalg import sqrtm
        from matplotlib import cm

        th = np.linspace(0, 2 * np.pi, 50)
        circle = np.array([np.sin(th), np.cos(th)])

        ell = (2 * sqrtm(covar) @ circle) + mean.reshape(2, 1)
        plt.plot(mean[0], mean[1], "o", ell[0, :], ell[1, :], ":", color="g")

    def draw_landmarks(self, landmark):
        self.draw_gaussians(landmark.mu, landmark.sigma)

    def draw_true_position(self):
        circle = plt.Circle((self.pos.pos[0, 0], self.pos.pos[1, 0]), 0.5)
        ax = plt.gca()
        ax.add_patch(circle)
        dx = self.pos.x() + 0.5 * math.cos(self.pos.orientation())
        dy = self.pos.y() + 0.5 * math.sin(self.pos.orientation())
        plt.plot([self.pos.x(), dx], [self.pos.y(), dy], c="k")

    def draw_particles(self):
        ax = plt.gca()
        x, y = [], []
        for particle in self.fastSlam.particles:
            x.append(particle.x())
            y.append(particle.y())
            # self.draw_landmarks(particle.landmarks[0])
        ax.scatter(x, y, c="r")

    def draw_lines(self, map_lines, color):
        for line in map_lines:
            x1, x2 = line.evaluate(0), line.evaluate(1)
            plt.plot([x1[0, 0], x2[0, 0]], [x1[1, 0], x2[1, 0]], c=color)

    def draw_observations(self, observations, color):
        p = self.pos[:2, :].reshape(2, 1)
        lines = []
        for i in range(observations.shape[1]):
            r, theta = observations[0, i], observations[1, i]
            x, y = r * math.cos(theta + self.pos.orientation()), r * math.sin(
                theta + self.pos.orientation()
            )
            p_temp = np.array([[x], [y]])
            lines.append(ParametricLine(p, p_temp))
        self.draw_lines(lines, color)

    def draw_map(self):
        self.draw_lines(self.map_lines, "b")
        self.draw_lines(self.landmark_lines, "k")
        
    def save_image(self, filename):
        plt.savefig(filename)
        plt.clf()

    def update_true_position(self, pos):
        self.pos = pos

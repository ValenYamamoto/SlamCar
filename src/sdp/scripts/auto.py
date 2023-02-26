from enum import Enum
import numpy as np

from utils import Moves

class State(Enum):
    STRAIGHT1 = 1
    STRAIGHT2 = 2
    STRAIGHT3 = 3
    STRAIGHT4 = 4
    TURN = 5

class PIDController:

    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d
        self.last_error = 0
        self.cum_error = 0

    def next(self, error):
        p = self.p * error
        self.cum_error += error
        i = self.i * self.cum_error
        diff = error - self.last_error
        d = self.d * diff
        self.last_error = error
        return p + i + d

class StraightPID(PIDController):
    def __init__(self, p, i, d, target=0):
        super().__init__(p, i, d)
        self.target = target

    def set_target(target):
        self.target = target

    def next(self, curr_value):
        error = target - curr_value
        return super().next(error)

def FSM(state, pos):
    if state == State.STRAIGHT1:
        if pos[1] >= 80:
            return State.TURN
        return State.STRAIGHT1
    if state == State.STRAIGHT2:
        if pos[0] <= 10:
            return State.TURN
        return State.STRAIGHT2
    if state == State.TURN:
        if pos[2] >= np.deg2rad(-90):
            return State.TURN
        return State.STRAIGHT2

def FSM_actions(state):
    if state == State.STRAIGHT1:
        return Moves.FORWARD
    if state == State.STRAIGHT2:
        return Moves.FORWARD
    if state == State.TURN:
        return Moves.LEFT

def auto_move_to_angle(move):
    if move == Moves.FORWARD:
        return 0
    elif move == Moves.BACKWARD:
        return 180 # TODO fix
    elif move == Moves.LEFT:
        return np.deg2rad(-35)
    elif move == Moves.RIGHT:
        return np.deg2rad(35)

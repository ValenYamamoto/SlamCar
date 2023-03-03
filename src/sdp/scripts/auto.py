from enum import Enum
import numpy as np

from utils import Moves

class State(Enum):
    STRAIGHT1 = 1
    STRAIGHT2 = 2
    STRAIGHT3 = 3
    STRAIGHT4 = 4
    TURN1 = 5
    TURN2 = 6
    TURN3 = 7
    TURN4 = 8

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

    def set_target(self, target):
        self.target = target
        self.cum_error = 0

    def next(self, curr_value):
        error = self.target - curr_value
        return super().next(error)


class FSM:
    def __init__(self, x1, x2):
        self.x1 = x1
        self.x2 = x2
        self.turn_start = 0

    def next_state(self, state, pos):
        if state == State.STRAIGHT1:
            if pos[1] >= self.x2 - 110:
                self.turn_start = 0
                return State.TURN1
            return State.STRAIGHT1

        if state == State.TURN1:
            delta = pos[2] - self.turn_start
            if delta >= np.deg2rad(-90):
                return State.TURN1
            return State.STRAIGHT2

        if state == State.STRAIGHT2:
            if pos[0] <= 110:
                return State.TURN2
            return State.STRAIGHT2

        if state == State.TURN2:
            if pos[2] <= 0:
                return State.TURN2
            return State.STRAIGHT3

        if state == State.STRAIGHT3:
            if pos[1] <= 110:
                self.turn_start = np.deg2rad(180)
                return State.TURN3
            return State.STRAIGHT3

        if state == State.TURN3:
            delta = pos[2] - self.turn_start
            if delta >= np.deg2rad(-90):
                return State.TURN3
            return State.STRAIGHT4

        if state == State.STRAIGHT4:
            if pos[0] >= self.x1 - 110:
                return State.TURN4
            return State.STRAIGHT4

        if state == State.TURN4:
            if pos[2] >= 0:
                return State.TURN4
            return State.STRAIGHT1


    def actions(self, state):
        if state in [State.STRAIGHT1, State.STRAIGHT2, State.STRAIGHT3, State.STRAIGHT4]:
            return Moves.FORWARD
        else:
            return Moves.LEFT

class FSMSimple:
    def __init__(self):
        self.turn_start = 0

    def next_state(self, state, pos):
        if state == State.STRAIGHT1:
            if pos[1] >= 90:
                self.turn_start = 0
                return State.TURN1
            return State.STRAIGHT1

        if state == State.TURN1:
            delta = pos[2] - self.turn_start
            if delta >= np.deg2rad(-90):
                return State.TURN1
            return State.STRAIGHT2

        if state == State.STRAIGHT2:
            return State.STRAIGHT2


    def actions(self, state):
        if state in [State.STRAIGHT1, State.STRAIGHT2, State.STRAIGHT3, State.STRAIGHT4]:
            return Moves.FORWARD
        else:
            return Moves.LEFT

    def get_pid_target(self, state):
        if state == State.STRAIGHT1:
            return 150
        if state == State.STRAIGHT2:
            return 150
        return 0

def auto_move_to_angle(move):
    if move == Moves.FORWARD:
        return 0
    elif move == Moves.BACKWARD:
        return 180 # TODO fix
    elif move == Moves.LEFT:
        return np.deg2rad(-35)
    elif move == Moves.RIGHT:
        return np.deg2rad(35)

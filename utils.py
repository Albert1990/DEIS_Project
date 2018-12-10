import numpy as np
from enum import Enum

class EncoderPulses:
    def __init__(self, left= 0, right= 0):
        # 4294967295
        self.left = left
        self.right = right

class RobotPos:
    def __init__(self, X= 0, Y = 0):
        self.X = int(float(X))
        self.Y = int(float(Y))
        self.Theta = 0
        self.C = np.zeros((3,3))

class IrSensors:
    def __init__(self, left= 0, center= 0, right= 0):
        self.left = left
        self.center = center
        self.right = right

class CollisionSensors:
    def __init__(self, left= 0, right= 0):
        self.left = left
        self.right = right

class RobotStatus(Enum):
    NOTHING = 0
    CHANGING_LANE = 1
    CHANGING_LANE_FINISHED = 2
    TRACKING_LANE = 3
    ROBOT_STOPPED = 4
    SIDE_DRIVING = 5

class Senarios(Enum):
    CHANGE_LANE = 1
    TAKE_OVER = 2
    LINE_FOLLOW = 3
    OBSTACLE_AVOIDANCE = 4
    SIDE_FORMATION = 5
    MERGING = 6

def parse_senario(sen):
        if sen == "LINE_FOLLOW":
            return Senarios.LINE_FOLLOW
        if sen == "CHANGE_LANE":
            return Senarios.CHANGE_LANE
        if sen == "TAKE_OVER":
            return Senarios.TAKE_OVER
        if sen == "OBSTACLE_AVOIDANCE":
            return Senarios.OBSTACLE_AVOIDANCE
        if sen == "SIDE_FORMATION":
            return Senarios.SIDE_FORMATION
        if sen == "MERGING":
            return Senarios.MERGING
        return Senarios.LINE_FOLLOW
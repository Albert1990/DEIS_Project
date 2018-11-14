import numpy as np

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
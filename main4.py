#!/usr/bin/env python

import red_board
import time
from robot import Robot
from line_tracker import LineTracker
from utils import RobotPos
import math
  
robot = Robot('/dev/ttyUSB0')
lineTracker = LineTracker(robot)
lineTracker.startLineTracker()
time.sleep(4)
lineTracker.stopLineTracker()
lineTracker.changeLane('right')
time.sleep(4)
lineTracker.startLineTracker()
time.sleep(4)
lineTracker.stopLineTracker()
robot.stop()
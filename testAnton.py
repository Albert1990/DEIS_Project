import line_tracker
import robot

import time

robot = robot.Robot('COM4')
print("hej")
lt = line_tracker.LineTracker(robot)


# -128 fram, -127 bak
# robot.setLeftMotorSpeed(-127)
#robot.setRightMotorSpeed(80)

##robot.setLeftMotorSpeed(-100)
##robot.setRightMotorSpeed(-100)
##time.sleep(2)
##robot.setLeftMotorSpeed(-130)
##robot.setRightMotorSpeed(-130)
##time.sleep(2)
##robot.setLeftMotorSpeed(100)
##robot.setRightMotorSpeed(100)
##time.sleep(2)
##robot.setLeftMotorSpeed(130)
##robot.setRightMotorSpeed(130)


time.sleep(5)

    

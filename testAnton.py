import line_tracker
import robot

import time

robot = robot.Robot('COM3')



lineTracker = line_tracker.LineTracker(robot)

lineTracker.startLineTracker()

#i = 0
# while True:
    # print(i, "i")
   
    # if( i == 200):
        # lineTracker.changeLane("left")
    
    
    # if( i == 400):
        # lineTracker.changeLane("right")
        
   
    # if( i == 600):
        # lineTracker.changeLane("left")

    # if( i == 900):
        # lineTracker.changeLane("left")

    # if( i == 1100):
        # lineTracker.changeLane("right")
    
##    if(i == 300):
##        lineTracker.changeLane("right")
##        
##    if(i == 500):
##        lineTracker.changeLane("left")
##    
    ##i +=1
    
   ## time.sleep(0.0001)
   

    

#time.sleep(2)

#robot.changeLane("left")


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


    

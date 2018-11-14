import line_tracker
import robot

import time
import threading

robot = robot.Robot('/dev/ttyUSB0')


lineTracker = line_tracker.LineTracker(robot)
print('start line tracking')
lineTracker.startLineTracker()



#time.sleep(5)
#lineTracker.changeLane("left")
#time.sleep(5)
#lineTracker.changeLane("right")

#time.sleep(12)
#lineTracker.stopLineTracker()
#print('stop line tracker')


#time.sleep(5)
#lineTracker.changeLane("left")
#time.sleep(5)
#lineTracker.changeLane("right")
#time.sleep(5)



# #i = 0
# while True:
    # #print(i, "i")
   
   # if( i == 20000):
    # lineTracker.changeLane("left")
    
    
    # # if( i == 400):
        # # lineTracker.changeLane("right")
        
   
    # # if( i == 600):
        # # lineTracker.changeLane("left")

    # # if( i == 900):
        # # lineTracker.changeLane("left")

    # # if( i == 1100):
        # # lineTracker.changeLane("right")
    
# #    if(i == 300):
# #        lineTracker.changeLane("right")
# #        
# #    if(i == 500):
# #        lineTracker.changeLane("left")
# #    
   # i +=1
    
   # time.sleep(0.0001)
   

    

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


#time.sleep(5)


    

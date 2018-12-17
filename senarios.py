from utils import RobotStatus
import time
from utils import Senarios

def lineFollow(lineTracker):
    lineTracker.turnUltrasonic(True)
    lineTracker.start()
    pass

def changeLane(robot, lineTracker):
    if robot.getStatus() == RobotStatus.NOTHING:
        robot.setStatus(RobotStatus.CHANGING_LANE)
        lineTracker.start()
        time.sleep(1)
        lineTracker.stop()
        print('########## starting change lane ###########')
        lineTracker.turnUltrasonic(False)
        lineTracker.changeLane('right')
    if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
        print('++++++ change lane stopped +++++')
        robot.setStatus(RobotStatus.NOTHING)
        lineTracker.turnUltrasonic(True)
        lineTracker.start()
        time.sleep(5)
        lineTracker.stop()
        robot.stop()

takeOverCounter = 0
def takeOver(robot, lineTracker):
    global takeOverCounter
    if robot.getStatus() == RobotStatus.NOTHING:
        print('takeover started !')
        lineTracker.start()
        time.sleep(1)
        lineTracker.stop()
        robot.setStatus(RobotStatus.CHANGING_LANE)
        print('right change lane started !')
        lineTracker.changeLane('right')
    if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
        if takeOverCounter == 0:
            print('right change lane finished !')
            robot.setStatus(RobotStatus.NOTHING)
            lineTracker.start()
            time.sleep(3)
            lineTracker.stop()
            robot.stop()
            robot.setStatus(RobotStatus.CHANGING_LANE)
            print('left change lane started !')
            lineTracker.changeLane('left')
            takeOverCounter += 1
        else:
            print('left change lane finished !')
            robot.setStatus(RobotStatus.NOTHING)
            lineTracker.start()
            time.sleep(3)
            lineTracker.stop()
            robot.stop()
            print('robot stopped !')


def obstacleAvoidance(robot, lineTracker):
    lineTracker.turnUltrasonic(False)
    dist = robot.getUltrasonicDistance()
    print('dist:', dist)
    if robot.getStatus() == RobotStatus.NOTHING:
        print('line tracking started')
        lineTracker.start()
        if dist < 30 and dist > 0:
            robot.setStatus(RobotStatus.CHANGING_LANE)
            lineTracker.stop()
            robot.slowdown()
            time.sleep(0.3)
            lineTracker.changeLane('right')
    if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
        robot.setStatus(RobotStatus.NOTHING)
        lineTracker.start()

def sideFormation(robot, lineTracker, partnerPosition, singleSenario = True):
    global takeOverCounter
    if robot.getStatus() == RobotStatus.NOTHING:
        lineTracker.start()
        time.sleep(1)
        lineTracker.stop()
        robot.setStatus(RobotStatus.CHANGING_LANE)
        lineTracker.changeLane('right')

    if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
        if not singleSenario and takeOverCounter != 0:
            print('soso')
        else:
            robot.setStatus(RobotStatus.SIDE_DRIVING)
            lineTracker.start()
            takeOverCounter += 1

    if robot.getStatus() == RobotStatus.SIDE_DRIVING:
        myPos = robot.getRobotPos()
        # print('myPos: (%d,%d)' % (myPos.X, myPos.Y))
        # print('partnerPos: (%d,%d)' % (partnerPosition.X, partnerPosition.Y))
        
        diffY = abs(partnerPosition.Y - myPos.Y)
        diffX = (partnerPosition.X - myPos.X)
        
        if myPos.Y > 300:
            diffX = -diffX
        
        # print("diffX:"+str(diffX))
        # print("diffY:"+str(diffY))
        if diffX <= 33:
            lineTracker.stop()
            robot.slowdown()
        else:
            lineTracker.start()

def merge(robot, lineTracker, partnerPosition, startStatus= RobotStatus.NOTHING, singleSenario= True):
    global takeOverCounter
    if robot.isLeader:
        if robot.getStatus() == startStatus:
            myPos = robot.getRobotPos()
            diffY = abs(partnerPosition.Y - myPos.Y)
            diffX = (partnerPosition.X - myPos.X)
            
            if myPos.Y > 300:
                diffX = -diffX
            
            # print("diffX:"+str(diffX))
            # print("diffY:"+str(diffY))
            if diffX < -60:
                lineTracker.stop()
                robot.setStatus(RobotStatus.CHANGING_LANE)
                lineTracker.changeLane('left')
            else:
                lineTracker.start()
        if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
            print(singleSenario, takeOverCounter)
            if not singleSenario and takeOverCounter != 1:
                return False
            robot.setStatus(RobotStatus.MERGING_FINISHED)
            print('###merging finished !!####')
            takeOverCounter += 1

    else: # follower
        if robot.getStatus() == startStatus:
            myPos = robot.getRobotPos()
            diffY = abs(partnerPosition.Y - myPos.Y)
            if diffY < 10 and diffY != 0:
                robot.setStatus(RobotStatus.MERGING_FINISHED)
                scenario = Senarios.LINE_FOLLOW
            diffX = (partnerPosition.X - myPos.X)
            
            if myPos.Y > 300:
                diffX = -diffX
            
            # print("diffX:"+str(diffX))
            # print("diffY:"+str(diffY))
            if diffX > 100:
                lineTracker.start()
            else:
                lineTracker.stop()
                robot.slowdown()

# def newLeader(robot, lineTracker, partnerPosition):
    
def resetTakeOverCounter():
    takeOverCounter = 0
from utils import RobotPos, EncoderPulses, IrSensors, CollisionSensors
import red_board
import odometry
import math
import threading
import time
import line_tracker

class Robot:
    def __init__(self, serialPort):
        self.robotPos = RobotPos()
        self.board = red_board.RedBoard(serialPort)
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.irSensors = IrSensors()
        self.collisionSensors = CollisionSensors()
        
        

        # create a thread that read sensor data always
        print("INit robot")
        t = threading.Thread(target=self._readSensors)
        t.start() 
        
        # create a thread that write motors data always (think about it more)

    def setRobotPos(self, robotPos):
        self.robotPos = robotPos
    
    def getRobotPos(self):
        return self.robotPos

    def setLeftMotorSpeed(self, speed):
        self.leftMotorSpeed = abs(speed)
        self.board.setLeftMotorSpeed(self.leftMotorSpeed, 1 if speed >= 0 else 0)

    def setRightMotorSpeed(self, speed):
        self.rightMotorSpeed = abs(speed)
        self.board.setRightMotorSpeed(self.rightMotorSpeed, 1 if speed >= 0 else 0)

    def _readSensors(self):
        while True:
            (leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor,leftColliderSensor, rightColliderSensor) =  self.board.readSensors()
            encoderPulses = EncoderPulses(leftEncoderValue, rightEncoderValue)
            
            robotPos = odometry.calculatePosition(encoderPulses, self.getRobotPos())
           
            if(robotPos):
                self.robotPos = robotPos
            self.irSensors = IrSensors(leftIRSensor, centerIRSensor, rightIRSensor)
            self.collisionSensors = CollisionSensors(leftColliderSensor, rightColliderSensor)
            time.sleep(0.001)

    def getIrSensors(self):
        return self.irSensors

    def getCollisionSensors(self):
        return self.collisionSensors
        
        
    
            
        
        
        
        


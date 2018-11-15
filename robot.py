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
        self.ultrasonicDistance = 0
        
        

        # create a thread that read sensor data always
        print("Init robot")
        t = threading.Thread(target=self._readSensors)
        t.start() 
        
        # create a thread that write motors data always (think about it more)

    def setRobotPos(self, robotPos):
        self.robotPos = robotPos
    
    def getRobotPos(self):
        return self.robotPos

    def setLeftMotorSpeed(self, speed):
        if self.leftMotorSpeed != speed:
            self.leftMotorSpeed = speed
            self.board.setLeftMotorSpeed(abs(self.leftMotorSpeed), 1 if self.leftMotorSpeed >= 0 else 0)

    def setRightMotorSpeed(self, speed):
        if self.rightMotorSpeed != speed:
            self.rightMotorSpeed = speed
            self.board.setRightMotorSpeed(abs(self.rightMotorSpeed), 1 if self.rightMotorSpeed >= 0 else 0)

    def _readSensors(self):
        while True:
            #print('_readSensors !')
            (leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor, distance) =  self.board.readSensors()
            #(leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor) =  self.board.readSensors()
            encoderPulses = EncoderPulses(leftEncoderValue, rightEncoderValue)
            
            robotPos = odometry.calculatePosition(encoderPulses, self.getRobotPos())
            
            if(robotPos):
                self.robotPos = robotPos
            self.irSensors = IrSensors(leftIRSensor, centerIRSensor, rightIRSensor)
            self.ultrasonicDistance = distance
            # print('distance: %d' % self.ultrasonicDistance)
            time.sleep(0.005)

    def getIrSensors(self):
        return self.irSensors

    def getUltrasonicDistance(self):
        return self.ultrasonicDistance

    def stop(self):
        self.setLeftMotorSpeed(0)
        self.setRightMotorSpeed(0)
        
        
    
            
        
        
        
        


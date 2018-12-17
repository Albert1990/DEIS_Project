import random
import time
# import threading
import thread
from utils import RobotStatus

class LineTracker:

    def __init__(self, robot):
        # P controller
        self.kp = 40
        self.kd = 80
        self.normalSpeed = 0
        self.changeLaneFraction = 0
        self.changeLaneSpeed = 0
        self.setPointSensor = 0
        self.LINETRESHOLD = 900    
        self.robot = robot
        self.lastSensorValue = 0 
        self.lastError = 0
        self.running = False
        self.isUltrasonicWorking = True
        
        # Distance controller
        self.setPointDistance = 1
        self.distanceKp = 1.5

        """ The control loop.
            Maybe this called from a timer. """
        print("thread started")
        # worker = threading.Thread(target=self.compute)
        # worker.start()
        thread.start_new_thread(self.compute, ())
        

    # the controller logic. computes the new controller output
    def compute(self):
        
        while True:
            # print('LineTracker')
            # get the sensor values from the robot
            leftIRSensor = self.robot.irSensors.left
            centerIRSensor = self.robot.irSensors.center
            rightIRSensor = self.robot.irSensors.right
            # print(leftIRSensor, centerIRSensor, rightIRSensor)
            
            sensorValue = self.__calculateAvrSensorValue(leftIRSensor, centerIRSensor, rightIRSensor)
            # print(sensorValue)

            error = self.setPointSensor*1.0 - sensorValue
            dError = error - self.lastError
            
            self.lastError = error
            
           
            # Distance
            if self.isUltrasonicWorking:
                ultrasonicDistance = self.robot.getUltrasonicDistance()
                # print('ultrasonicDistance:', ultrasonicDistance)
                if(ultrasonicDistance > 0):
                    distanceError = ultrasonicDistance - self.setPointDistance*1.0 
                    self.normalSpeed = self.distanceKp*distanceError
            
            if(self.normalSpeed < 0 ):
                self.normalSpeed = 0
                
            if(self.normalSpeed > 100):
                self.normalSpeed = 100
                
                
            #if(self.normalSpeed > 30):
            leftSpeed = int(self.normalSpeed - (self.kp*error + self.kd*dError))
            rightSpeed = int(self.normalSpeed + (self.kp*error + self.kd*dError))
            #else:
             #   leftSpeed = 0
              #  rightSpeed = 0
            
            if(leftSpeed > 254):
                leftSpeed = 254
            
            if(rightSpeed > 254):
                rightSpeed = 254
                
            if(leftSpeed < 0):
                leftSpeed = 0
            
            if(rightSpeed < 0):
                rightSpeed = 0
           
            # set the speed of the motors
            if( self.running == True ):
                # print("running", leftSpeed, rightSpeed)
                # print('set motors spppp')
                self.robot.setLeftMotorSpeed(leftSpeed)
                self.robot.setRightMotorSpeed(rightSpeed)
                
           # if( self.robot.getUltrasonicDistance() <= 10 ):
                #print("under 10")
            #    self.robot.setLeftMotorSpeed(0)
             #   self.robot.setRightMotorSpeed(0)
                
              #  self.stopLineTracker()
            
            
            #else:
             #   self.startLineTracker()
            
            
            time.sleep(0.01)

    def turnUltrasonic(self, flag):
        self.isUltrasonicWorking = flag

    def start(self):
        # print("line tracking started")
        self.running = True
        # start linetracking

    def stop(self):
        # print("line tracking stopped")
        self.running = False
		
        # stop linetracking

    
    def set_kp(self, k_p):
        self.k_p = k_p
        
        
    def __calculateAvrSensorValue(self, leftIRSensor, centerIRSensor, rightIRSensor):
        steeringValue = 0
        nrOfSensorsActive = 0
        
        if(self.sensorIsActive(leftIRSensor)):
            steeringValue -= 1
            nrOfSensorsActive += 1
            
        if(self.sensorIsActive(centerIRSensor)):
            steeringValue += 0
            nrOfSensorsActive += 1
        
        
        if(self.sensorIsActive(rightIRSensor)):
            steeringValue += 1
            nrOfSensorsActive += 1
            
        if( self.isOutSideOfTrack(leftIRSensor, centerIRSensor, rightIRSensor) ):
            steeringValue = self.lastSensorValue
                         
        else:
            steeringValue = steeringValue*1.0 / nrOfSensorsActive*1.0

        self.lastSensorValue = steeringValue
 
        #print(steeringValue)
        return steeringValue
        
        
        
        
    def sensorIsActive(self, sensorValue):
        return sensorValue > self.LINETRESHOLD
        
        
    def isOutSideOfTrack(self, leftIRSensor, centerIRSensor, rightIRSensor):
        return (not self.sensorIsActive(leftIRSensor)) and (not self.sensorIsActive(centerIRSensor)) and (not self.sensorIsActive(rightIRSensor))
        
        
    def changeLane(self, direction):
        thread.start_new_thread(self.__changeLane, (direction,))

    def __changeLane(self, direction):
        if(direction == "left"):
            self.robot.setLeftMotorSpeed(self.normalSpeed - self.changeLaneFraction)
            self.robot.setRightMotorSpeed(self.normalSpeed + self.changeLaneFraction)
        elif(direction == "right"):
            self.robot.setLeftMotorSpeed(self.normalSpeed + self.changeLaneFraction)
            self.robot.setRightMotorSpeed(self.normalSpeed - self.changeLaneFraction)
        
        # Wait until we are out of the line and the correct side of the line
        if( direction == "left"): 
            while( not ( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) and self.lastSensorValue > 0)):
                # print("wait until we are out of the line")
                time.sleep(0.01)
            
                
            # Wait until the other line is catched
            while( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) or self.lastSensorValue < 0 ):
                # print(self.lastSensorValue)
                # print("outside of line")
                self.robot.setLeftMotorSpeed(self.changeLaneSpeed)
                self.robot.setRightMotorSpeed(self.changeLaneSpeed)
                time.sleep(0.01)
                
                
        elif( direction == "right"):
            while( not (self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) and self.lastSensorValue < 0)):
                # print("wait until we are out of the line")
                time.sleep(0.01)
                
            # Wait until the other line is catched
            while( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) or self.lastSensorValue > 0 ):
                # print(self.lastSensorValue)
                self.robot.setLeftMotorSpeed(self.changeLaneSpeed)
                self.robot.setRightMotorSpeed(self.changeLaneSpeed)
                time.sleep(0.01)
                # print("outside of line")
      
        self.robot.setStatus(RobotStatus.CHANGING_LANE_FINISHED)
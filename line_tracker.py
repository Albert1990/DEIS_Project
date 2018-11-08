import random
import time
import threading

class LineTracker:

    def __init__(self, robot):
		
        # P controller
        self.kp = 40
        self.normalSpeed = 80
        self.setPointSensor = 0
        self.LINETRESHOLD = 900    
        self.robot = robot
        self.lastSensorValue = 0 
        self.running = False

        """ The control loop.
            Maybe this called from a timer. """
        print("thread started")
        worker = threading.Thread(target=self.compute)
        worker.start()
        


    # the controller logic. computes the new controller output
    def compute(self):
        
        while True:
            # get the sensor values from the robot
            leftIRSensor = self.robot.irSensors.left
            centerIRSensor = self.robot.irSensors.center
            rightIRSensor = self.robot.irSensors.right
            
            sensorValue = self.calculateAvrSensorValue(leftIRSensor, centerIRSensor, rightIRSensor)

            error = self.setPointSensor*1.0 - sensorValue

            leftSpeed = int((self.normalSpeed - self.kp*error))
            rightSpeed = int(self.normalSpeed + self.kp*error)
           
            # set the speed of the motors
            if( self.running == True ):
                self.robot.setLeftMotorSpeed(leftSpeed)
                self.robot.setRightMotorSpeed(rightSpeed)
                
            
            time.sleep(0.0001)

        

    def startLineTracker(self):
        print("line tracking started")
        self.running = True
        # start linetracking

    def stopLineTracker(self):
        print("line tracking stopped")
        self.running = False
		
        # stop linetracking

    
    def set_kp(self, k_p):
        self.k_p = k_p
        
        
    def calculateAvrSensorValue(self, leftIRSensor, centerIRSensor, rightIRSensor):
    
        #print(leftIRSensor, centerIRSensor, rightIRSensor)
    
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
            
        #print(steeringValue)
        self.lastSensorValue = steeringValue
        
        
        return steeringValue
        
        
        
        
    def sensorIsActive(self, sensorValue):
        return sensorValue > self.LINETRESHOLD
        
        
    def isOutSideOfTrack(self, leftIRSensor, centerIRSensor, rightIRSensor):
        return (not self.sensorIsActive(leftIRSensor)) and (not self.sensorIsActive(centerIRSensor)) and (not self.sensorIsActive(rightIRSensor))
        
        
    def changeLane(self, direction):
        print("changing line to", direction)
        self.stopLineTracker()
        
        if(direction == "left"):
            self.robot.setLeftMotorSpeed(self.normalSpeed - (20))
            self.robot.setRightMotorSpeed(self.normalSpeed + (20))
        elif(direction == "right"):
            self.robot.setLeftMotorSpeed(self.normalSpeed + (20))
            self.robot.setRightMotorSpeed(self.normalSpeed - (20))

        
        # Wait until we are out of the line and the correct side of the line
        if( direction == "left"): 
            while( not ( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) and self.lastSensorValue > 0)):
                print("wait until we are out of the line")
                pass
                
            # Wait until the other line is catched
            while( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) or self.lastSensorValue < 0 ):
                print(self.lastSensorValue)
                print("outside of line")
                
                
        elif( direction == "right"):
            while( not (self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) and self.lastSensorValue < 0)):
                print("wait until we are out of the line")
                pass
                
            # Wait until the other line is catched
            while( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) or self.lastSensorValue > 0 ):
                print(self.lastSensorValue)
                print("outside of line")
      
        
           
        self.startLineTracker()
        
        
            
           
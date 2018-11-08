import random
import time


class LineTracker:

    

    

    def __init__(self, robot):
		
        # P controller
        self.kp = 40
        self.normalSpeed = 80
        self.setPointSensor = 0
        self.LINETRESHOLD = 900
        
        
        self.robot = robot
        self.lastSensorValue = 0 
        self.start()

        """ The control loop.
            Maybe this called from a timer. """
        i = 0
        while True:
            self.compute()
            #print("haha")  
            #print( "position:", self.robot.robotPos.X, self.robot.robotPos.Y, self.robot.robotPos.Theta)
            if( i == 30):
                self.changeLane("left")
            
            
            if(i == 90):
                self.changeLane("right")
                
            if(i == 140):
                self.changeLane("left")
            
            i +=1
            print(i, "i")
            time.sleep(0.0001)


    # the controller logic. computes the new controller output
    def compute(self):
        
        # get the sensor values from the robot
        
        leftIRSensor = self.robot.irSensors.left
        centerIRSensor = self.robot.irSensors.center
        rightIRSensor = self.robot.irSensors.right
        
        sensorValue = self.calculateAvrSensorValue(leftIRSensor, centerIRSensor, rightIRSensor)

        """"
        Maybe we need to calculate a sensor average value with
        the sensor values as input.
        """ 
        error = self.setPointSensor*1.0 - sensorValue

        leftSpeed = int((self.normalSpeed - self.kp*error))
        rightSpeed = int(self.normalSpeed + self.kp*error)
       
        # set the speed of the motors
        if( self.running == True ):
            self.robot.setLeftMotorSpeed(leftSpeed)
            self.robot.setRightMotorSpeed(rightSpeed)

        

    def start(self):
        self.running = True
        # start linetracking

    def stop(self):
        print("line tracking stopped")
        self.running = False
		
        # stop linetracking

    
    def set_kp(self, k_p):
        self.k_p = k_p
        
        
    def calculateAvrSensorValue(self, leftIRSensor, centerIRSensor, rightIRSensor):
    
        print(leftIRSensor, centerIRSensor, rightIRSensor)
    
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
        
        
        return steeringValue
        
        
        
        
    def sensorIsActive(self, sensorValue):
        return sensorValue > self.LINETRESHOLD
        
        
    def isOutSideOfTrack(self, leftIRSensor, centerIRSensor, rightIRSensor):
        return (not self.sensorIsActive(leftIRSensor)) and (not self.sensorIsActive(centerIRSensor)) and (not self.sensorIsActive(rightIRSensor))
        
        
    def changeLane(self, direction):
        print("changing line to", direction)
        self.stop()
        
        if(direction == "left"):
            self.robot.setLeftMotorSpeed(self.normalSpeed - (40))
            self.robot.setRightMotorSpeed(self.normalSpeed + (30))
        elif(direction == "right"):
            self.robot.setLeftMotorSpeed(self.normalSpeed + (30))
            self.robot.setRightMotorSpeed(self.normalSpeed - (40))

        
        # Wait until we are out of the line 
        while( not self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) && ):
            pass
      
        # Wait until the other line is catched
        while( self.isOutSideOfTrack(self.robot.irSensors.left, self.robot.irSensors.center, self.robot.irSensors.right) ):
            print("outside of line")
           
        self.start()
        
        
            
           
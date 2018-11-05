import random
import time


class LineTracker:

    

    

    def __init__(self, robot):
		
        # P controller
        self.kp = 80
        self.normalSpeed = 80
        self.setPointSensor = 0
        self.LINETRESHOLD = 900
        
        self.lastSensorValue = 0
        
        self.robot = robot

        self.lastSensorValue = 0
        
        self.start()

        """ The control loop.
            Maybe this called from a timer. """
        while True:
            self.compute()

            time.sleep(1)


    # the controller logic. computes the new controller output
    def compute(self):
	
        if( self.running ):
        
            # get the sensor values from the robot
            #sensorValue = robot.getSensorValues()
            # sensorValue = random.randint(-1, 1)
            
            self.robot._readSensors()
            
            leftIRSensor = self.robot.irSensors.left
            centerIRSensor = self.robot.irSensors.center
            rightIRSensor = self.robot.irSensors.right
            
            
            
            sensorValue = self.calculateAvrSensorValue(leftIRSensor, centerIRSensor, rightIRSensor)
            
            print(sensorValue)

            """"
            Maybe we need to calculate a sensor average value with
            the sensor values as input.
            """
                
            error = self.setPointSensor*1.0 - sensorValue
            print(error, "error")

            leftSpeed = int(-(self.normalSpeed - self.kp*error))
            rightSpeed = int(self.normalSpeed + self.kp*error)
            
            #leftSpeed = abs(leftSpeed)
            #rightSpeed = abs(rightSpeed)
            print(leftSpeed, rightSpeed)

            # set the speed of the motors
            self.robot.setLeftMotorSpeed(leftSpeed)
            self.robot.setRightMotorSpeed(rightSpeed)
            #robot.setMotorSpeed(leftSpeed, rightSpeed)
            
            #print(leftSpeed)
            #print(rightSpeed)
		
        

    def start(self):
        self.running = True
        # start linetracking

    def stop(self):
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
            
        
        
        
        if( (not self.sensorIsActive(leftIRSensor)) and (not self.sensorIsActive(centerIRSensor)) and (not self.sensorIsActive(rightIRSensor))):
            steeringValue = self.lastSensorValue 
            
        else:
            steeringValue = steeringValue*1.0 / nrOfSensorsActive*1.0
            
        
        self.lastSensorValue = steeringValue
        
        
        return steeringValue
        
        
        
        
    def sensorIsActive(self, sensorValue):
        return sensorValue > self.LINETRESHOLD

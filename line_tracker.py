import random
import time


class LineTracker:

    

    

    def __init__(self):
		
        # P controller
        self.kp = 80
        self.normalSpeed = 80
        self.setPointSensor = 0

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
            sensorValue = random.randint(-1, 1)

            """"
            Maybe we need to calculate a sensor average value with
            the sensor values as input.
            """
                
            error = self.setPointSensor - sensorValue

            leftSpeed = -(self.normalSpeed - self.kp*error)
            rightSpeed = self.normalSpeed + self.kp*error

            # set the speed of the motors
            #robot.setMotorSpeed(leftSpeed, rightSpeed)
            
            print(leftSpeed)
            print(rightSpeed)
		
        

    def start(self):
        self.running = True
        # start linetracking

    def stop(self):
        self.running = False
		
        # stop linetracking

    
    def set_kp(self, k_p):
        self.k_p = k_p
        

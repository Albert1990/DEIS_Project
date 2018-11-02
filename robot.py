from utils import RobotPos, EncoderPulses, IrSensors, CollisionSensors
import red_board
import odometry

class Robot:
    def __init__(self):
        self.robotPos = RobotPos()
        self.board = red_board.RedBoard('/dev/tty.usbserial-AH05K013')
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.irSensors = IrSensors()
        self.collisionSensors = CollisionSensors()

        # create a thread that read sensor data always
        # create a thread that write motors data always (think about it more)

    def setRobotPos(self, robotPos):
        self.robotPos = robotPos
    
    def getRobotPos(self):
        return self.robotPos

    def setLeftMotorSpeed(self, speed):
        self.leftMotorSpeed = speed
        self.board.setLeftMotorSpeed(speed, 1 if speed >= 0 else 0)

    def setRightMotorSpeed(self, speed):
        self.rightMotorSpeed = speed
        self.board.setRightMotorSpeed(speed, 1 if speed >= 0 else 0)

    def _readSensors(self):
        (leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor,leftColliderSensor, rightColliderSensor) =  self.board.readSensors()
        encoderPulses = EncoderPulses(leftEncoderValue, rightEncoderValue)
        self.robotPos = odometry.calculate_position(encoderPulses)
        self.irSensors = IrSensors(leftIRSensor, centerIRSensor, rightIRSensor)
        self.collisionSensors = CollisionSensors(leftColliderSensor, rightColliderSensor)

    def getIrSensors(self):
        return self.irSensors

    def getCollisionSensors(self):
        return self.collisionSensors


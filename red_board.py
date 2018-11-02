import serial
import time

class RedBoard:
    port = ''
    ser = ''
    commandSize = 15

    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port = self.port, baudrate = 9600)
        # here the arduino is going to restart so we have to give it a time
        time.sleep(2)

    def setLeftMotorSpeed(self, speed, isPositive):
        packet = bytearray([0x01, 0x01, speed, isPositive])
        return self.sendData(packet)
    
    def setRightMotorSpeed(self, speed, isPositive):
        packet = bytearray([0x01, 0x02, speed, isPositive])
        return self.sendData(packet)

    def setMode(self, mode):
        packet = bytearray([0x01, 0x03, mode])
        return self.sendData(packet)

    def readSensors(self):
        packet = bytearray([0x01, 0x04])
        self.sendData(packet)
        sensorsData = self.ser.read(self.commandSize)

        leftEncoderValue = self.bytesToLong(sensorsData[2:6])
        rightEncoderValue = self.bytesToLong(sensorsData[6:10])
        leftIRSensor = sensorsData[10]
        centerIRSensor = sensorsData[11]
        rightIRSensor = sensorsData[12]
        leftColliderSensor = sensorsData[13]
        rightColliderSensor = sensorsData[14]
        return (leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor,leftColliderSensor, rightColliderSensor)
        # print(leftEncoderValue)
        # print(rightEncoderValue)
        # print(sensorsData)
        # print(len(sensorsData))
        # now we have to parse the received data into a dict

    def sendData(self, data):
        if not self.ser.isOpen():
            print('Serial port is busy !')
            return False

        # stuffing dummy data inside the packet to meet command standard size
        if len(data) < self.commandSize:
            for i in range(len(data), self.commandSize):
                data.append(0x01)
        # print(data)
        # print(len(data))
        # print('------------')

        self.ser.write(data)
        return True

    def bytesToLong(self, bytesArr):
        result = 0
        for b in bytesArr:
            result = result * 256 + int(b)
        return result

    def intToBytes(self, value, length):
        result = []

        for i in range(0, length):
            result.append(value >> (i * 8) & 0xff)

        result.reverse()

        return result
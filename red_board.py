import serial
import time
import struct

class RedBoard:
    port = ''
    ser = ''
    commandSize = 20

    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port = self.port, baudrate = 250000)
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
        #print('s1')
        sensorsData = list(self.ser.read(self.commandSize))
        #print('tt')
        #print(sensorsData)

        leftEncoderValue = self.bytesToLong(sensorsData[2:6])
        rightEncoderValue = self.bytesToLong(sensorsData[6:10])
        leftIRSensor = self.bytesToLong(sensorsData[10:12])
        centerIRSensor = self.bytesToLong(sensorsData[12:14])
        rightIRSensor = self.bytesToLong(sensorsData[14:16])
        # distance = self.bytesToLong(sensorsData[16:18])
        
        # leftColliderSensor = sensorsData[16]
        # rightColliderSensor = sensorsData[17]
        #return (leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor, distance)
        return (leftEncoderValue, rightEncoderValue, leftIRSensor, centerIRSensor, rightIRSensor)
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
        #print("send success:", data)
        return True

    def bytesToLong(self, bytesArr):
        # print('soso')
        result = 0
        for b in bytesArr:
            # print(struct.unpack("B", b))
            result = result * 256 + (struct.unpack("B", b)[0])
        return result

    def intToBytes(self, value, length):
        result = []

        for i in range(0, length):
            result.append(value >> (i * 8) & 0xff)

        result.reverse()

        return result
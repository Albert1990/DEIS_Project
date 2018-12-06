import cv2
import threading
import time

class Camera:
    def __init__(self):
        self.obstacleAvailable = False
        self.camera = cv2.VideoCapture(0)
        self.lowerHsv = (105, 100, 50)
        self.higherHsv = (109, 255, 255)
        self.kernel3 = (5,5)

        if not self.camera.isOpened():
            print('camera is not opened !')
        else:
            t = threading.Thread(target=self._readFrame)
            t.start() 

    def _readFrame(self):
        while True:
            _, frame = self.camera.read()
            frame = cv2.resize(frame, (320, 240))
            hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsvImage, self.lowerHsv, self.higherHsv)
            mask = cv2.dilate(mask, self.kernel3)
            mask = cv2.erode(mask, self.kernel3)
            M = cv2.moments(mask)
            # calculate x,y coordinate of center
            if M["m00"] == 0:
                cX =0
                cY =0
            else:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            print('obstacle: %d, %d' % (cX, cY))
            if cX > 166 and cY > 110:
                self.obstacleAvailable = True
            else:
                self.obstacleAvailable = False
            time.sleep(0.1)
        
    def isObstacleAvailable(self):
        return self.obstacleAvailable
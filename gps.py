import cv2
import numpy as np
import time


class Gps:
    def __init__(self):
        self.originalCameraMatrix=np.array([[540.92809281, 0, 310.49944426], [0, 540.23283561, 227.12758261], [0, 0, 1]],np.float64)
        self.optimalCameraMatrix = np.array([[457.13543701, 0, 306.81114822], [0, 493.90228271, 223.85634775], [0, 0, 1]],np.float64)
        self.distortionCoefficients = np.array([[-4.01495171e-01, 1.93524711e-01, -1.71371113e-03, -1.11895861e-04, -3.48628258e-02]],np.float64)
        self.pixel_to_mm = 4.9445
        self.error_per_mm = 0.0000124
        self.x0 = 190
        self.y0 = 120

    def get_xy_coordinates(self,gps_image, timestamp):
        undistorted_gps_image = cv2.undistort(gps_image, self.originalCameraMatrix, self.distortionCoefficients, None, self.optimalCameraMatrix)

        #cv2.imwrite("New_empty.jpg", undistorted_gps_image)
        #cv2.imshow("new", undistorted_gps_image)
        cv2.waitKey(0)
        # find the r,c of the robot-------------------------------------------------
        img_hsv = cv2.cvtColor(undistorted_gps_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
        kernel1 = np.ones((3, 3), np.uint8)
        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel1)
        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel1)
        mu1 = cv2.moments(mask1, True)
        front = [-1,-1]
        if mu1["m00"] > 0:
            front = [int(mu1["m10"] / mu1["m00"]), int(mu1["m01"] / mu1["m00"])]
        front_c = front[0]
        front_r = front[1]

        # Lego
        # lower_blue = np.array([100,50,50])
        # upper_blue = np.array([110,255,255])

        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])

        mask2 = cv2.inRange(img_hsv, lower_blue, upper_blue)
        kernel2 = np.ones((5, 5), np.uint8)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernel2)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel2)
        mu2 = cv2.moments(mask2, True)
        #debug
        '''
        cv2.imshow("mask1",mask1)
        cv2.imshow("mask2",mask2)
        k=0
        cv2.waitKey()
        if k == 27:
            cv2.destroyAllWindows()
        '''

        rear = [-1, -1]
        if mu2["m00"] > 0:
            rear = [int(mu2["m10"] / mu2["m00"]), int(mu2["m01"] / mu2["m00"])]

        rear_c = rear[0]
        rear_r = rear[1]

        #mask = mask1 + mask2
        if mu1["m00"]==0 or mu2["m00"]==0:
            return 0,0, [0, 0, 0]
        x1 = rear_c
        y1 = 480 - rear_r

        x2 = front_c
        y2 = 480 - front_r

        robot_r = rear_r + int(float(front_r - rear_r) / 2)
        robot_c = rear_c + int(float(front_c - rear_c) / 2)

        theta = np.arctan(float(y2 - y1) / float(x2 - x1))

        theta_deg = theta * (180 / np.pi)

        # print "x,y,Theta=(" + str(robot_c) + "," + str(robot_r) + "," + str(theta_deg) + ")"

        # flip r (Y points upward)
        r = 480-robot_r
        c = robot_c
        x = (c*self.pixel_to_mm)+self.x0
        y = (r*self.pixel_to_mm)+self.y0
        return 1, timestamp, [x, y, theta_deg]


#Class Usage
gps = Gps()
img = cv2.imread("5.jpg")
v,t,X = gps.get_xy_coordinates(img,time.time())

print "time:"+str(t)
print "x,y,Theta=(" + str(X[0]) + "," + str(X[1]) + "," + str(X[2]) + ")"


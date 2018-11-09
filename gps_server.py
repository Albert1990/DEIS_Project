import cv2
import sys
from subprocess import call

print('Camera app has been started')
cameraGetImageCommand = "wget --user root --password r3dM1lk -q -O current.jpg http://192.168.1.8/axis-cgi/jpg/image.cgi"
while True:
	call(cameraGetImageCommand, shell=True)
	print('k1')
	img = cv2.imread("current.jpg")
	print('k2')
	cv2.imshow('frame1', img)
	if cv2.waitKey(1) == 27:
		break


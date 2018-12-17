from talker import Talker 
import rospy
from std_msgs.msg import String
import cv2
import cv_bridge
from cv_bridge import CvBridgeError
import time

rospy.init_node('Group_1', anonymous=True)
talker = Talker(10)
time.sleep(1)
for i in range(5):
    talker.pub_action('a', 5, 2,5,'hello')
    time.sleep(1)
print('done')
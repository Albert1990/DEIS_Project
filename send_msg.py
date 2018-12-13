from talker import Talker 
import rospy
from std_msgs.msg import String
import cv2
import cv_bridge
from cv_bridge import CvBridgeError


rospy.init_node('Group_1', anonymous=True)
talker = Talker(10)
talker.pub_action('T', 3, -1, 2, 'msg1 Samoor')
talker.pub_feedback('W', 1, 8, 'msg2 Samoor')

print('#######message has been sent !#########')
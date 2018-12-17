#!/usr/bin/env python
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

import rospy
from std_msgs.msg import String
from talker import Talker 
import thread

class Listener():
	def __init__(self):
		rospy.Subscriber('heartbeat_channel', String, self.heartbeat_receive)
		rospy.Subscriber('action_channel', String, self.action_receive)
		rospy.Subscriber('feedback_channel', String, self.feedback_receive)
		#rospy.init_node('Group_1', anonymous=True)
		self.action_queue = []
		self.feedback_queue = []
		thread.start_new_thread(self.spin, ())
	
	
	def spin(self):
		rospy.spin()

	# The method the be called from outside to get the latest action
	def receive_action(self):
		if len(self.action_queue) > 0:
			return self.action_queue.pop(0)
	def receive_feedback(self):
		if len(self.feedback_queue) > 0:
			return self.feedback_queue.pop(0)
	
	def heartbeat_receive(self, data):
		if data.data.split()[2] == '5' or data.data.split()[2] == '6':
			rospy.loginfo(rospy.get_caller_id() + 'I heard heartbeat %s', data.data)

	def action_receive(self, data):
		# print("action receive")
		# print('data:', data.data)
		if data.data.split()[4] == '5' or data.data.split()[4] == '6':
			# rospy.loginfo(rospy.get_caller_id() + 'I heard action %s', data.data)
			#talker = Talker(10)
			action_id = data.data.split()[1]
			source_robot_id = data.data.split()[2]
			target_platoon_id = data.data.split()[3]
			msg = data.data.split()[5]
			action_message  = ActionMessage(action_id, source_robot_id, target_platoon_id, msg)
			self.action_queue.append(action_message)
	
	def feedback_receive(self, data):
		if data.data.split()[3] == '5'or data.data.split()[3] == '6':
			# rospy.loginfo(rospy.get_caller_id() + 'I heard feedback %s', data.data)
			action_id = data.data.split()[1]
			source_robot_id = data.data.split()[2]
			msg = data.data.split()[4]
			feedback_message  = FeedbackMessage(action_id, source_robot_id, msg)
			self.feedback_queue.append(feedback_message)

class ActionMessage:
	
	def __init__(self, action_id, source_robot_id, target_platoon_id, msg):
		self.action_id = action_id
		self.source_robot_id = source_robot_id
		self.target_platoon_id = target_platoon_id
		self.msg = msg

class FeedbackMessage:
	
	def __init__(self, action_id, source_robot_id, msg):
		self.action_id = action_id
		self.source_robot_id = source_robot_id
		self.msg = msg
			
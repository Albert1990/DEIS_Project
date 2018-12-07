#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import thread

class Talker():
	def __init__(self, rate):
		self.pub_hb = rospy.Publisher('heartbeat_channel', String, queue_size=10)
		self.pub_ac = rospy.Publisher('action_channel', String, queue_size=10)
    		self.pub_fb = rospy.Publisher('feedback_channel', String, queue_size=10)
		self.rate = rospy.Rate(rate) # 10hz
		thread.start_new_thread(self.rate.sleep,())

	        
	def heartbeat (self, platoon_id=-1,robot_id=0,robot_type=0,lane_id=-1,robot_pose=(0,0,0,0),speed=0):
	   	timestamps = str(rospy.get_time())
		platoon_id = str(platoon_id)
		robot_id = str(robot_id)
		robot_type = str(robot_type)
		lane_id = str(lane_id)
		robot_pose = str(robot_pose)
		speed = str(speed)
		string = timestamps+' '+platoon_id+' ' +robot_id+' '+robot_type+' '+lane_id+' '+robot_pose+' '+speed
		return string
	
	def action(self, action_id,source_robot_id,target_platoon_id,target_robot_id,msg):
	   	timestamps = str(rospy.get_time())
		source_robot_id = str(source_robot_id)
		target_platoon_id = str(target_platoon_id)
		target_robot_id = str (target_robot_id)
		string = timestamps+' '+action_id+' ' +source_robot_id+' '+target_platoon_id+' '+target_robot_id+' '+msg
		return string
	
	def feedback(self, action_id, my_robot_id, other_robot_id,msg):
		timestamps =str(rospy.get_time())
		my_robot_id = str(my_robot_id)
		other_robot_id = str(other_robot_id)
		string = timestamps+' '+action_id+' '+my_robot_id+' '+other_robot_id+' '+msg
		return string

	def pub_heartbeat(self, platoon_id,robot_id,robot_type,lane_id,robot_pose,speed):
		# 1,5,1,3,(2,5,4,8),15
		hello_str = self.heartbeat(platoon_id,robot_id,robot_type,lane_id,robot_pose,speed)
		rospy.loginfo(hello_str)
    		self.pub_hb.publish(hello_str)

	def pub_action(self, action_id,source_robot_id,target_platoon_id,target_robot_id,msg):
		#'a', 5, 2,7,'hello'
		hello_str = self.action(action_id,source_robot_id,target_platoon_id,target_robot_id,msg)	
		rospy.loginfo(hello_str)
		self.pub_ac.publish(hello_str)

	def pub_feedback(self, action_id, my_robot_id, other_robot_id,msg):
		#'d',5, 8, 'hello'
	    	hello_str = self.feedback( action_id, my_robot_id, other_robot_id,msg)
		rospy.loginfo(hello_str)
		self.pub_fb.publish(hello_str)
		

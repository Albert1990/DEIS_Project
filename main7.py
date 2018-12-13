#!/usr/bin/env python
import argparse
import red_board
import time
from robot import Robot
from line_tracker import LineTracker
from utils import RobotPos
from utils import RobotStatus
from utils import Senarios
from utils import parse_senario
import math
import thread
from talker import Talker 
from listener import Listener

import rospy
from std_msgs.msg import String
import cv2
import cv_bridge
from cv_bridge import CvBridgeError
import senarios

robotName = 'Zingo'
partnerPosition = RobotPos()
scenario = Senarios.LINE_FOLLOW

if __name__ == "__main__":
    robot = Robot('/dev/ttyUSB0')
    lineTracker = LineTracker(robot)
    if robotName == 'Zingo':
        rospy.init_node('Group_1_Zingo', anonymous=True)
    else:
        rospy.init_node('Group_1_Ringo', anonymous=True)
    listener1 = Listener()
    talker = Talker(10)

    parser = argparse.ArgumentParser(description='Red robot program')
    parser.add_argument('--robotName', type=str, default='Zingo')
    parser.add_argument('--isLeader', type=bool, default=True)
    parser.add_argument('--senario', type=str, default='LINE_FOLLOW')
    args = parser.parse_args()
    robotName = args.robotName
    robot.isLeader = args.isLeader
    scenario = parse_senario(args.senario)
    if robotName == 'Zingo':
        lineTracker.normalSpeed = 120
        lineTracker.changeLaneFraction = 50
        lineTracker.changeLaneSpeed = 70
    else:
        lineTracker.normalSpeed = 180
        lineTracker.changeLaneFraction = 70
        lineTracker.changeLaneSpeed = 70

    def gpsInfoHandlar(data):
        global robot
        global partnerPosition
        receivedData = data.data
        # print('receivedData: %s' % (receivedData))
        # [72 266 0.560329420160072 0 0.403142816922575;115.760363550632 410.006777530987 0.974541946123153 1 0.617652217032975;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;188.26307348859 401.237015686746 -0.98185802727514 8 0.763166075364706;-1 -1 -1 -1 -1]
        if len(receivedData) > 0:
            positions_data = receivedData.split(';')
            robot1_pos_chuncks = positions_data[5].split(' ')
            robot1Pos = RobotPos(robot1_pos_chuncks[1], robot1_pos_chuncks[0])
            robot2_pos_chuncks = positions_data[6].split(' ')
            robot2Pos = RobotPos(robot2_pos_chuncks[1], robot2_pos_chuncks[0])
            # print('robot1Pos: (%d,%d)' % (robot1Pos.X, robot1Pos.Y))
            # print('robot2Pos: (%d,%d)' % (robot2Pos.X, robot2Pos.Y))

            if robotName == 'Zingo':
                robot.setRobotPos(robot2Pos)
                partnerPosition = robot1Pos           
            else:
                robot.setRobotPos(robot1Pos)
                partnerPosition = robot2Pos


    # init gps listener
    rospy.Subscriber("josefoutput", String, gpsInfoHandlar)
    thread.start_new_thread(rospy.spin, ())
    while True:
        action_message = listener1.receive_action()
        feedback_message = listener1.receive_feedback()
        # print('msg_action_message:', action_message)
        # print('msg_feedback_message:', feedback_message)

        if action_message is not None:
            print('rec action_msg:', action_message)
            if action_message.action_id == 'm':
                print('#####merge rec#####')
                if robotName == 'Ringo':
                    print('Ringo in Merge mode now !!')
                    talker.pub_feedback('m',5, 6, 'msg 1')
                    scenario = Senarios.MERGING
                    robot.isLeader = False

        if feedback_message is not None:
            print('rec feedback_msg:', feedback_message)
            if feedback_message.action_id == 'm':
                if robotName == 'Zingo':
                    print('Zingo in Merge mode now !!')
                    scenario = Senarios.MERGING
                    robot.isLeader = True
                

        if scenario == Senarios.LINE_FOLLOW:
            senarios.lineFollow(lineTracker)

        if scenario == Senarios.CHANGE_LANE:
            senarios.changeLane(robot, lineTracker)
        
        if scenario == Senarios.TAKE_OVER:
            senarios.takeOver(robot, lineTracker)

        if scenario == Senarios.OBSTACLE_AVOIDANCE:
            senarios.obstacleAvoidance(robot, lineTracker)

        if scenario == Senarios.SIDE_FORMATION:
            senarios.sideFormation(robot, lineTracker, partnerPosition)

        if scenario == Senarios.MERGING:
            senarios.merge(robot, lineTracker, partnerPosition)

        if scenario == Senarios.NEW_LEADER:
            print('#NEW_LEADER#')
            lineTracker.turnUltrasonic(True)
            if robotName == 'Zingo':
                talker.pub_action('m', 6, -1, 5, 'msg1')
                print('#######message has been sent !#########')
            senarios.sideFormation(robot, lineTracker, partnerPosition)
            senarios.merge(robot, lineTracker, partnerPosition, RobotStatus.SIDE_DRIVING, False)
                
                
        time.sleep(0.2)
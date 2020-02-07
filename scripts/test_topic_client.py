#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from tf import TransformListener
import geometry_msgs.msg
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from dimr_kuka.msg import DimrControl


if __name__ == '__main__':

    pub = rospy.Publisher("kuka_bridge", DimrControl, queue_size=10)

    rospy.init_node("test_topic_client", anonymous=True)



    rospy.loginfo("Beginning publishing to kuka bridge ...")


    # On créé notre message à envoyer
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 1.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = -0.45
    pose_goal.position.z = 0.1

    test = DimrControl()
    test.brick_pose = pose_goal

    pub.publish(test)

    rospy.loginfo("Stopped manipulation")





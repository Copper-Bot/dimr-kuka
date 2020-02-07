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
from dimr_kuka.srv import SendToolTo,SendToolToRequest,SendToolToResponse


if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_service_client", anonymous=True)
    rospy.wait_for_service('kuka_bridge_service')

    send_tool_to = rospy.ServiceProxy('kuka_bridge_service', SendToolTo)

    rospy.loginfo("Beginning calling service ...")


    # On créé notre message à envoyer
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 1.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.45
    pose_goal.position.z = 0.1

    test = SendToolToRequest()
    test.brick_pose = pose_goal

    resp = send_tool_to.call(test)

    rospy.loginfo(resp.success.data)

    rospy.loginfo("Stopped manipulation")





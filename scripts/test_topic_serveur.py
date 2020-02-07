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


def handle_dimrcontrol_message(data):
    rospy.loginfo("Message DimrControl has been received.")
    move_group.set_pose_target(data.brick_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo("finish")
    rospy.set_param("/kuka/busy", False)

if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_topic_serveur", anonymous=True)
    rospy.loginfo("Beginning demo")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(1)  # on attend un peu avant de publier
    sub = rospy.Subscriber("kuka_bridge", DimrControl, handle_dimrcontrol_message)
    rospy.sleep(1)
    rospy.loginfo("topic kuka_bridge subscribed and ready to process")
    rospy.loginfo("adding objects")

    # # palette
    # base_palette = geometry_msgs.msg.PoseStamped()
    # base_palette.header.frame_id = "base"  # lancer un tf view pour choisir le bon nom !
    # base_palette.pose.orientation.w = 0.0
    # base_palette.pose.position.x = -0.095
    # base_palette.pose.position.y = 0.0
    # base_palette.pose.position.z = -0.0625
    # palette_name = "base_palette"
    # scene.add_box(palette_name, base_palette, size=(0.64, 0.80, 0.125))

    # Sol
    sol = geometry_msgs.msg.PoseStamped()
    sol.header.frame_id = "base"  # lancer un tf view pour choisir le bon nom !
    sol.pose.orientation.w = 0.0
    sol.pose.position.x = 0.0
    sol.pose.position.y = 0.0
    sol.pose.position.z = -0.125
    sol_name = "base_sol"
    scene.add_box(sol_name, sol, size=(4, 4, 0))

    # Box
    # box = geometry_msgs.msg.PoseStamped()
    # box.header.frame_id = "base"
    # box.pose.position.x = 0.625  # .30
    # box.pose.position.y = 0.01  # .30
    # box.pose.position.z = 0.07  # .30
    # box.pose.orientation.w = 1.0
    # box_name = "box"
    # scene.add_box(box_name, box, size=(0.68, 0.45, 0.39))

    rospy.sleep(1)  # on attend un peu avant de publier

    # Exit
    rospy.spin()
    rospy.loginfo("Stopped manipulation")

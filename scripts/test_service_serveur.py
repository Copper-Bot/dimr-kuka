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


def add_send_tool_server():
    s = rospy.Service('kuka_bridge_service', SendToolTo, handle_send_tool)
    rospy.loginfo("send tool service add")

def handle_send_tool(req):
    rospy.loginfo("Service SendToolTo has been called.")
    move_group.set_pose_target(req.brick_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    resp = SendToolToResponse()
    resp.success.data = True
    resp.status = 1
    return resp

if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("kuka_bridge_node", anonymous=True)
    rospy.loginfo("Beginning demo")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(1)  # on attend un peu avant de publier
    add_send_tool_server()
    rospy.loginfo("adding objects")

    # palette
    base_palette = geometry_msgs.msg.PoseStamped()
    base_palette.header.frame_id = "base"  # lancer un tf view pour choisir le bon nom !
    base_palette.pose.orientation.w = 0.0
    base_palette.pose.position.x = -0.095
    base_palette.pose.position.y = 0.0
    base_palette.pose.position.z = -0.0625
    palette_name = "base_palette"
    scene.add_box(palette_name, base_palette, size=(0.64, 0.80, 0.125))

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
    box = geometry_msgs.msg.PoseStamped()
    box.header.frame_id = "base"
    box.pose.position.x = 0.625  # .30
    box.pose.position.y = 0.01  # .30
    box.pose.position.z = 0.07  # .30
    box.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box, size=(0.68, 0.45, 0.39))

    rospy.sleep(1)  # on attend un peu avant de publier

    # Exit
    rospy.spin()
    rospy.loginfo("Stopped manipulation")





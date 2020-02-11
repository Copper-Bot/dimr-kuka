#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from tf import TransformListener
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
#from moveit_msgs.srv import GetPositionIK
#from moveit_msgs.srv import GetPositionIKRequest
#from moveit_msgs.srv import GetPositionIKResponse
#from sensor_msgs.msg import Image
#import matplotlib.pyplot as plt
#from cv_bridge import CvBridge, CvBridgeError
from math import pi
from std_msgs.msg import Empty, String
from moveit_commander.conversions import pose_to_list
from dimr_kuka.msg import DimrControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray

delta = 0.05


ws_center_x = 0.20
ws_center_y = -0.20
ws_center_z = 0.6

ws_radius_x = 0.5
ws_radius_y = 0.6
ws_radius_z = 0.6

def handle_controller_message(data):
    if rospy.get_param("/kuka/manual"):
        effector_init = move_group.get_current_pose().pose
        effector_target = effector_init
        effector_target.position.x += delta * data.buttons[4] - delta * data.buttons[5]
        effector_target.position.y += delta * data.buttons[12] - delta * data.buttons[11]
        effector_target.position.z += delta * data.buttons[13] - delta * data.buttons[14]

        buttons_state = sum(data.buttons)

        if buttons_state != 0:
            if (effector_target.position.x > (ws_center_x - ws_radius_x)) and (
                    effector_target.position.x < (ws_center_x + ws_radius_x)) \
                    and (effector_target.position.y > (ws_center_y - ws_radius_y)) and (
                    effector_target.position.y < (ws_center_y + ws_radius_y)) \
                    and (effector_target.position.z > (ws_center_z - ws_radius_z)) and (
                    effector_target.position.z < (ws_center_z + ws_radius_z)):
                rospy.loginfo("Moving effector manually...")
                move_group.set_planning_time(0.1)
                move_group.set_pose_target(effector_target)
                plan = move_group.go(wait=True)
                if not plan :
                    rospy.logwarn("Can't move effector there ! (outside joints)")
                move_group.stop()
                move_group.clear_pose_targets()
            else:
                rospy.logwarn("Can't move effector there ! (outside workspace)")

        # goal = PoseStamped()
        # goal.header.frame_id = "base_link"
        # goal.header.stamp = rospy.Time(0.0)
        # goal.pose = effector_target
        #
        # plan_pub.publish(Empty())
        #
        # pub_goal.publish(goal)
        #
        # execute_pub.publish(Empty())
        # update_start_state_pub.publish(Empty())

        #
        # vib = JoyFeedback()
        # vib.type = 1
        # vib.id = 0
        # vib.intensity = 0.6
        # vibarr = JoyFeedbackArray()
        # vibarr.array.append(vib)
        # pub.publish(vibarr)


if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_joy_bridge", anonymous=True)
    rospy.loginfo("Beginning demo")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(1)  # on attend un peu avant de publier
    sub = rospy.Subscriber("joy", Joy, handle_controller_message, queue_size=1)
    pub = rospy.Publisher("/joy/set_feedback/", JoyFeedbackArray, queue_size=1)
    # pub_goal = rospy.Publisher("/rviz/moveit/move_marker/goal_tool0", PoseStamped, queue_size=5)
    #
    # plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
    # execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty, queue_size=5)
    # update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty, queue_size=5)

    # rospy.sleep(1)
    # rospy.loginfo("topic kuka_bridge subscribed and ready to process")
    # rospy.loginfo("adding objects")

    # # palette
    # base_palette = geometry_msgs.msg.PoseStamped()
    # base_palette.header.frame_id = "base"  # lancer un tf view pour choisir le bon nom !
    # base_palette.pose.orientation.w = 0.0
    # base_palette.pose.position.x = -0.095
    # base_palette.pose.position.y = 0.0
    # base_palette.pose.position.z = -0.0625
    # palette_name = "base_palette"
    # scene.add_box(palette_name, base_palette, size=(0.64, 0.80, 0.125))
    #
    # # Sol
    # sol = PoseStamped()
    # sol.header.frame_id = "base"  # lancer un tf view pour choisir le bon nom !
    # sol.pose.orientation.w = 0.0
    # sol.pose.position.x = 0.0
    # sol.pose.position.y = 0.0
    # sol.pose.position.z = -0.01
    # sol_name = "base_sol"
    # scene.add_box(sol_name, sol, size=(4, 4, 0))

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
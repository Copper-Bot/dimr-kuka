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
from math import pi
from std_msgs.msg import Empty, String
from moveit_commander.conversions import pose_to_list
from dimr_kuka.msg import DimrControl
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray

delta = 0.05

ws_center_x = 0.13
ws_center_y = -0.20
ws_center_z = 0.6

ws_radius_x = 0.52
ws_radius_y = 0.55
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



if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("joystick", anonymous=True)
    rospy.loginfo("Beginning demo")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(1)  # on attend un peu avant de publier
    sub = rospy.Subscriber("joy", Joy, handle_controller_message, queue_size=1)
    pub = rospy.Publisher("/joy/set_feedback/", JoyFeedbackArray, queue_size=1)


    rospy.sleep(1)  # on attend un peu avant de publier

    # Exit
    rospy.spin()
    rospy.loginfo("Stopped manipulation")

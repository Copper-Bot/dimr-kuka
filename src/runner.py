#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import sleep
import rospy
import sys
import numpy as np
import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import tf

from math import pi
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

class Runner(object):
    def __init__(self, brick_number, brick_type, x_base, y_base, z_base):
        self.brick_number = brick_number
        self.brick_type = brick_type
        #coordinate of the runner in the base frame
        self.pose = Pose()
        self.pose.position.x = x_base
        self.pose.position.y = y_base
        self.pose.position.z = z_base
        self.pose.orientation.x = 0
        self.pose.orientation.y = 0
        self.pose.orientation.z = 0
        self.pose.orientation.w = 1

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

from enum import Enum
from wall import Wall

class Small(Enum):
    #brick dimensions
    height = 0.1
    width = 0.09
    depth = 0.1

class Big(Enum):
    height = 0.1
    width = 0.18
    depth = 0.1

class Brick(object):
    sb_created = 0
    bb_created = 0

    #static position for all Brick's instances
    #TODO : input the taking coordinates for big bricks
    big_taking_pose = Pose()
    big_taking_pose.position.x = 0
    big_taking_pose.position.y = 0
    big_taking_pose.position.y = 0
    big_taking_pose.orientation.x = 0
    big_taking_pose.orientation.y = 0
    big_taking_pose.orientation.z = 0
    big_taking_pose.orientation.w = 0

    #TODO : input the taking coordinates for small bricks
    small_taking_pose = Pose()
    small_taking_pose.position.x = 0
    small_taking_pose.position.y = 0
    small_taking_pose.position.y = 0
    small_taking_pose.orientation.x = 0
    small_taking_pose.orientation.y = 0
    small_taking_pose.orientation.z = 0
    small_taking_pose.orientation.w = 0

    def __init__(self,type,layer,column):
        self.type = type
        #position of the left-hand bottom corner of the brick
        self.placing_pose = Pose()
        self.placing_pose.position.x = column*self.type.width
        self.placing_pose.position.y = Wall.placing_pose.position.y
        self.placing_pose.position.z = layer*self.type.height
        self.placing_pose.orientation.x = Wall.placing_pose.orientation.x
        self.placing_pose.orientation.y = Wall.placing_pose.orientation.y
        self.placing_pose.orientation.z = Wall.placing_pose.orientation.z
        self.placing_pose.orientation.w = Wall.placing_pose.orientation.w

        self.taking_pose = Pose()
        if(self.type == Small()):
            sb_created +=1
            self.taking_pose.position.x = Wall.taking_pose_s2.position.x
            self.taking_pose.position.y = Wall.taking_pose_s2.position.y
            self.taking_pose.position.z = Wall.taking_pose-s2.position.z
            self.taking_pose.orientation.x = Wall.taking_pose_s2.orientation.x
            self.taking_pose.orientation.y = Wall.taking_pose_s2.orientation.y
            self.taking_pose.orientation.z = Wall.taking_pose_s2.orientation.z
            self.taking_pose.orientation.w = Wall.taking_pose_s2.orientation.w
        else:
            bb_created += 1
            if(bb_created <= Wall.)
        self.is_placed = False
    
    def remove_brick_from_layer():
        self.is_placed = False
        

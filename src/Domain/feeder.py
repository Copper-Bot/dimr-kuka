#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import sleep
# import rospy
import sys
import numpy as np
import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
# import tf

from math import pi
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
# from brick import Type

from enum import Enum

class Type(Enum):
    #brick's width
    small = 0.09
    big = 0.18

class Feeder(object):
    def __init__(self,id, brick_capacity, brick_type, brick_height, pose):
        self.id = id
        self.brick_capacity = brick_capacity
        self.brick_count = 0
        self.brick_type = brick_type.name
        #coordinate of the feeder in the base frame (middle point of the bottom brick of the feeder)
        self.height = brick_capacity * brick_height
        if(self.brick_type == Type.big.name):
            self.width = Type.big.value
        else:
            self.width = Type.small.value
        self.depth = brick_height
        self.pose = pose
        self.bricks = []


    def next_available_pose(self, brick_height):
        next_pose = Pose()
        next_pose.position.x = self.pose.position.x
        next_pose.position.y = self.pose.position.y
        next_pose.position.z = self.pose.position.z + self.brick_count*brick_height
        next_pose.orientation.x = self.pose.orientation.x
        next_pose.orientation.y = self.pose.orientation.y
        next_pose.orientation.z = self.pose.orientation.z
        next_pose.orientation.w = self.pose.orientation.w
        return next_pose


    def fill(self,bricks):
        for b in self.bricks:
            self.add_brick(b)
    
    def empty(self):
        for b in self.bricks:
            self.remove_brick()
    
    def add_brick(self,brick):
        print("add")
        if(self.brick_count + 1 <= self.brick_capacity):
            self.bricks.append(brick)
            self.brick_count += 1
            return True
        else:
            print("The feeder is already full. Choose an another one")
            return False
    
    def remove_brick(self):
        #pop() returns an error if the list is empty, pop(0) returns the first brick in the list and pop() the last one
        if(not self.is_empty()):
            self.brick_count -= 1
            return self.bricks.pop()
        else:
            print("Feeder is already empty")
            return None

    def is_empty(self):
        return self.brick_count == 0
    
    def is_filled_up(self):
        return self.brick_count == self.brick_capacity
    
    def to_string(self):
        print '      Brick type : {0} \n      Brick capacity : {1} \n      Brick count : {2} \n'.format(self.brick_type,self.brick_capacity,self.brick_count)
        for b in self.bricks:
            b.to_string()


brick_height = 0.1

 #TODO : input the taking coordinates for the b6 runner
taking_pose_b6 = Pose()
taking_pose_b6.position.x = 0
taking_pose_b6.position.y = -0.8
taking_pose_b6.position.z = 0.05
taking_pose_b6.orientation.x = 0
taking_pose_b6.orientation.y = 1
taking_pose_b6.orientation.z = 0
taking_pose_b6.orientation.w = 0

f1 = Feeder(1, 6, Type.big, brick_height, taking_pose_b6)

#TODO : input the taking coordinates for the b5 runner
taking_pose_b5 = Pose()
taking_pose_b5.position.x = -0.3
taking_pose_b5.position.y = -0.8
taking_pose_b5.position.z = 0.05
taking_pose_b5.orientation.x = 0
taking_pose_b5.orientation.y = 1
taking_pose_b5.orientation.z = 0
taking_pose_b5.orientation.w = 0

f2 = Feeder(2, 5, Type.big, brick_height, taking_pose_b5)

#TODO : input the taking coordinates for the s2 runner
taking_pose_s2 = Pose()
taking_pose_s2.position.x = 0.3
taking_pose_s2.position.y = -0.8
taking_pose_s2.position.z = 0.05
taking_pose_s2.orientation.x = 0
taking_pose_s2.orientation.y = 1
taking_pose_s2.orientation.z = 0
taking_pose_s2.orientation.w = 0

f3 = Feeder(3, 2, Type.small, brick_height, taking_pose_s2)

global feeders
feeders = [f1,f2,f3]

for f in feeders:
    f.to_string()
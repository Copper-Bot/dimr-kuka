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

class Feeder(object):
    def __init__(self,brick_capacity, brick_type, pose):
        self.brick_capacity = brick_capacity
        self.brick_count = 0
        self.brick_type = brick_type
        #coordinate of the feeder in the base frame
        self.pose = pose
        self.bricks = []

    def fill(bricks):
        for b in bricks:
            self.add_brick(b)
    
    def empty():
        for b in self.bricks:
            self.remove_brick()
    
    def add_brick(brick):
        if(self.brick_count + 1 <= self.brick_capacity):
            self.bricks.append(brick)
            self.brick_count += 1
            return True
        else:
            print("The runner is already full. Choose an another one")
            return False
    
    def remove_brick(self):
        #pop(0) returns an error if the list is empty
        if(not self.is_empty()):
            self.brick_count -= 1
            self.bricks.pop(0)
        else:
            print("Feeder is already empty")

    def is_empty(self):
        return self.brick_count == 0
    
    def to_string(self):
        print 'Brick type : {0} \n Brick capacity : {1} \n Brick count : {2} \n'.format(self.brick_type,self.brick_capacity,self.brick_count)

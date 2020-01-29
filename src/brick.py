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
    #static properties for all Brick's instances

    def __init__(self,type,layer,column):
        #type of the brick
        self.type = type
        #position of the middle of the brick, dynamically attributed by the user when a click is detected on the interface
        self.placing_pose = Pose()
        self.placing_pose.position.x = column*self.type.width
        self.placing_pose.position.y = Wall.placing_pose.position.y
        self.placing_pose.position.z = layer*self.type.height
        self.placing_pose.orientation.x = Wall.placing_pose.orientation.x
        self.placing_pose.orientation.y = Wall.placing_pose.orientation.y
        self.placing_pose.orientation.z = Wall.placing_pose.orientation.z
        self.placing_pose.orientation.w = Wall.placing_pose.orientation.w
        #position of the middle of the brick
        self.taking_pose = Pose()
        self.feeder = None
        self.is_placed = False
    
    def find_right_feeder(self, feeders):
        #put the brick in the right feeder and attribute its corresponding taking pose
        r = 0
        feeder_found = False
        while(feeder_found == False and r < len(feeders))
            feeder = feeders[r]
            if(self.type == Small() and feeder.brick_type == Small()):
                if(feeder.add_brick()):
                    feeder_found = True
                    self.feeder = feeder
                    self.set_taking_pose(feeder.pose)
            elif(self.type == Big() and feeder.brick_type == Big()):
                if(feeder.add_brick()):
                    feeder_found = True
                    self.feeder = feeder
                    self.set_taking_pose(feeder.pose)
            r += 1
    
    def set_taking_pose(self,pose):
        self.taking_pose.position.x = pose.position.x
        self.taking_pose.position.y = pose.position.y
        self.taking_pose.position.z = pose.position.z
        self.taking_pose.orientation.x = pose.orientation.x
        self.taking_pose.orientation.y = pose.orientation.y
        self.taking_pose.orientation.z = pose.orientation.z
        self.taking_pose.orientation.w = pose.orientation.w

    def move_to_wall(self, feeders):
        self.is_placed = True
        self.find_right_feeder(feeders)
    
    def remove_from_wall(self):
        self.is_placed = False
        move_to_feeder()
    
    def move_to_feeder(self):
        if(self.feeder != None):
            n = self.feeder.brick_count + 1
            self.taking_pose.position.x = self.feeder.pose.position.x
            self.taking_pose.position.y = self.feeder.pose.position.y
            self.taking_pose.position.z = self.feeder.pose.position.z + n*self.type.height
            self.taking_pose.orientation.x = self.feeder.pose.orientation.x
            self.taking_pose.orientation.y = self.feeder.pose.orientation.y
            self.taking_pose.orientation.z = self.feeder.pose.orientation.z
            self.taking_pose.orientation.w = self.feeder.pose.orientation.w
            self.feeder.add_brick(self)


    #in the order :
    #create feeders
    #create wall
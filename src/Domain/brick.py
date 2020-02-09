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

from enum import Enum

class Type(Enum):
    #brick's width
    small = 0.09
    big = 0.18

class Brick(object):
    #static properties for all Brick's instances
    def __init__(self,type,layer,column):
        #type of the brick
        self.num_column = column
        self.num_layer = layer
        self.type = type.name
        self.height = 0.1
        self.width = type.value
        self.depth = 0.1
        #position of the middle of the brick, dynamically attributed by the user when a click is detected on the interface
        #TODO : input the placing coordinates for the wall (middle of the left-hand bottom brick of the wall)
        self.wall_pose = Pose()
        self.compute_wall_pose(layer, column)
        #position of the middle of the brick in the feeder
        self.feeder_pose = Pose()
        self.feeder = None
        self.is_placed = False
    
    def compute_wall_pose(self, layer, column):
        self.wall_pose.position.x = 0.7
        self.wall_pose.position.y = -0.41 + column*Type.big.value
        self.wall_pose.position.z = layer*self.height + self.height/2
        self.wall_pose.orientation.x = 0
        self.wall_pose.orientation.y = 1
        self.wall_pose.orientation.z = 0.0
        self.wall_pose.orientation.w = 0
        if(self.num_layer%2 == 1):
            if(column == 0):
                self.wall_pose.position.y -= Type.small.value/2
            elif(self.type == Type.small.name):
                self.wall_pose.position.y -= 3*Type.small.value/2
            else:
                self.wall_pose.position.y -= Type.small.value
            

    def find_right_feeder(self, feeders):
        #put the brick in the right feeder and attribute its corresponding taking pose
        r = 0
        feeder_found = False
        while(feeder_found == False and r < len(feeders)):
            feeder = feeders[r]
            # feeder.to_string()
            print(self.type)
            if(self.type == Type.small.name and feeder.brick_type == Type.small.name):
                if(feeder.add_brick(self)):
                    feeder_found = True
                    self.feeder = feeder
                    self.feeder_pose = self.feeder.next_available_pose(self.height)
            elif(self.type == Type.big.name and feeder.brick_type == Type.big.name):
                if(feeder.add_brick(self)):
                    feeder_found = True
                    self.feeder = feeder
                    self.feeder_pose = self.feeder.next_available_pose(self.height)
            r += 1
        print 'feeder found : {0} --> feeder capacity : {1} and brick count : {2}'.format(feeder_found,self.feeder.brick_capacity, self.feeder.brick_count)

    def add_to_wall(self):
        if(self.is_placed == False):
            self.is_placed = True
            self.feeder.remove_brick()
            return True
        else:
            print("The brick is already in the wall")
            return False


    def remove_from_wall(self):
        if(self.is_placed == True):
            self.is_placed = False
            self.add_to_feeder()
        else:
            print("The brick isn't placed in the wall")

    def add_to_feeder(self):
        if(self.feeder != None):
            self.feeder_pose = self.feeder.next_available_pose(self.height)
            self.feeder.add_brick(self)

    def to_string(self):
        print "Brick number : {0}".format(self.num_column)
        if(self.feeder != None):
            print '   Type : {0}\n   Is placed : {1}\n   Feeder : '.format(self.type, self.is_placed)
            print(self.feeder.to_string())
        else:
            print '   Type : {0}\n   Is placed : {1}\n   Feeder : {2}\n '.format(self.type, self.is_placed, self.feeder)

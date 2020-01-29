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

from layer import Layer

class Wall(object):
    #TODO : input the placing coordinates for the wall (left-hand bottom corner of the wall)
    placing_pose = Pose()
    placing_pose.position.x = 0
    placing_pose.position.y = 0
    placing_pose.position.y = 0
    placing_pose.orientation.x = 0
    placing_pose.orientation.y = 0
    placing_pose.orientation.z = 0
    placing_pose.orientation.w = 1

    # #TODO : input the taking coordinates for the b6 runner
    # taking_pose_b6 = Pose()
    # taking_pose_b6.position.x = 0
    # taking_pose_b6.position.y = -0.5
    # taking_pose_b6.position.y = 0
    # taking_pose_b6.orientation.x = 0
    # taking_pose_b6.orientation.y = 0
    # taking_pose_b6.orientation.z = 0
    # taking_pose_b6.orientation.w = 1

    # #TODO : input the taking coordinates for the b5 runner
    # taking_pose_b5 = Pose()
    # taking_pose_b5.position.x = -0.3
    # taking_pose_b5.position.y = -0.5
    # taking_pose_b5.position.y = 0
    # taking_pose_b5.orientation.x = 0
    # taking_pose_b5.orientation.y = 0
    # taking_pose_b5.orientation.z = 0
    # taking_pose_b5.orientation.w = 1

    # #TODO : input the taking coordinates for the s2 runner
    # taking_pose_s2 = Pose()
    # taking_pose_s2.position.x = 0.3
    # taking_pose_s2.position.y = -0.5
    # taking_pose_s2.position.y = 0
    # taking_pose_s2.orientation.x = 0
    # taking_pose_s2.orientation.y = 0
    # taking_pose_s2.orientation.z = 0
    # taking_pose_s2.orientation.w = 1

    #Wall main features
    layer_number = 3
    column_number = 4
    sb_number = 2
    bb_number = 11

    def __init__(self, feeders):
        self.layers = [None]*layer_number
        self.is_finished = False
        self.fill()
        self.fill_feeders(feeders)
    
    def fill(self):
        for l in range(layer_number):
            self.layers[l] = Layer(l) 

    def count_placed_bricks(self):
        #count the number of small and big bricks placed in the wall
        #number of small bricks
        sb_number = 0
        #number of big bricks
        bb_number = 0
        for layer in layers:
            bricks_nb = layer.count_placed_bricks()
            sb_number += bricks_nb[0]
            bb_number += bricks_nb[1]
        return sb_number, bb_number

    def destroy(self):
        if(self.is_empty() == False):
            for l in layers:
                l.destroy()

    def buid(self):
        if(self.is_empty() == False):
            for l in layers:
                l.build()

    def at(self,layer,column):
        return layers[layer].bricks[column]
    
    def is_empty(self):
        is_empty = True
        l = 0
        while(is_empty == True and l < n):
            if(self.layers[l] != None):
                if(self.layers[l].is_placed == True):
                    is_empty = False
        return is_empty

    def is_filled_up(self):
        is_filled_up = True
        l = 0
        while(is_filled_up == True and l < n):
            if(self.layers[l] != None):
                if(self.layers[l].is_placed == False):
                    is_filled_up = False
        return is_filled_up
    
    def fill_feeders(self, feeders):
        if(self.is_empty() == False):
            for l in layers:
                l.fill_feeders(feeders)

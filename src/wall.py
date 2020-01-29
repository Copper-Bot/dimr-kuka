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

from layer import Layer

class Wall(object):

    def __init__(self, feeders, layer_number, column_number, bb_number, sb_number):
        self.layer_number = layer_number
        self.column_number = column_number
        self.bb_number = bb_number
        self.sb_number = sb_number
        self.layers = [None]*layer_number
        self.is_finished = False
        self.fill()
        self.fill_feeders(feeders)
    
    def fill(self):
        for l in range(self.layer_number):
            self.layers[l] = Layer(l,self.column_number) 

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
        while(is_empty == True and l < len(self.layers)):
            if(self.layers[l] != None):
                if(self.layers[l].is_empty() == False):
                    is_empty = False
                else:
                    l+=1
        return is_empty

    def is_filled_up(self):
        is_filled_up = True
        l = 0
        while(is_filled_up == True and l < len(self.layers)):
            if(self.layers[l] != None):
                if(self.layers[l].is_filled_up() == False):
                    is_filled_up = False
                else:
                    l+=1
        return is_filled_up
    
    def fill_feeders(self, feeders):
        if(self.is_empty() == False):
            for l in layers:
                l.fill_feeders(feeders)

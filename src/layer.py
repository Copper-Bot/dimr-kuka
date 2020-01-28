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

from brick import Small, Big, Brick
from wall import Wall

class Layer(object):
    def __init__(self,num):
        #even layer : 0, odd layer : 1
        self.num = num
        self.parity = num % 2
        self.bricks = []
        self.is_filled_up = False
        fill_layer()

    def fill_layer(self):
        if(self.parity == 0):
            for b in range(Wall.column_number_bottom_layer):
                self.bricks.append(Brick(Big(),self.num,b))
        else:
            self.bricks.append(Brick(Small(),self.num,0))
            for k in range(1,Wall.column_number_bottom_layer):
                self.bricks.append(Brick(Big(),self.num,k))
            self.bricks.append(Brick(Small(),self.num,Wall.column_number_bottom_layer))
        
    def count_placed_bricks(self):
        #count the bricks placed in the layer
        sb_number = 0
        bb_number = 0
        for brick in bricks:
            if(brick.is_placed):
                if(brick.type = Small()):
                    sb_number+=1
                else:
                    bb_number+=1
        return sb_number,bb_number


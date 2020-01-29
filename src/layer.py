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

class Layer(object):
    column_number = 4
    def __init__(self,num):
        #even layer : 0, odd layer : 1
        self.num = num
        self.parity = num % 2
        self.bricks = []
        self.is_filled_up = False
        self.fill_layer()

    def fill_layer(self):
        if(self.parity == 0):
            for b in range(self.column_number):
                self.bricks.append(Brick(Big(),self.num,b))
        else:
            self.bricks.append(Brick(Small(),self.num,0))
            for k in range(1,self.column_number):
                self.bricks.append(Brick(Big(),self.num,k))
            self.bricks.append(Brick(Small(),self.num,self.column_number))
        
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
    
    def is_empty(self):
        n = len(self.bricks)
        if(n != 0):
            is_empty = True
            b = 0
            while(is_empty == True and b < n):
                if(self.bricks[b].is_placed == True):
                    is_empty = False
        return is_empty

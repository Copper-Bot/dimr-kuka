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

from brick import Type, Brick

class Layer(object):
    def __init__(self,num,capacity):
        #even layer : 0, odd layer : 1
        self.num = num
        self.parity = num % 2
        self.capacity = capacity
        self.bricks = [None] * capacity
        self.fill()

    def fill(self):
        if(self.parity == 0):
            for b in range(self.capacity):
                print(b)
                self.bricks[b] = Brick(Type.big,self.num,b)
        else:
            print(0)
            self.bricks[0] = Brick(Type.small,self.num,0)
            for k in range(1,self.capacity-1):
                print(k)
                self.bricks[k] = Brick(Type.big,self.num,k)
            print(self.capacity-1)
            self.bricks[self.capacity-1] = Brick(Type.small,self.num,self.capacity-1)

    def count_placed_bricks(self):
        #count the bricks placed in the layer
        sb_number = 0
        bb_number = 0
        for brick in self.bricks:
            if(brick.is_placed):
                if(brick.type == Type.small.name):
                    sb_number+=1
                else:
                    bb_number+=1
        return sb_number,bb_number

    def is_empty(self):
        is_empty = True
        b = 0
        while(is_empty == True and b < len(self.bricks)):
            if(self.bricks[b] != None):
                if(self.bricks[b].is_placed == True):
                    is_empty = False
                else:
                    b += 1
        return is_empty

    def is_filled_up(self):
        is_filled_up = True
        b = 0
        while(is_filled_up == True and b < len(self.bricks)):
            if(self.bricks[b] != None):
                if(self.bricks[b].is_placed == False):
                    is_filled_up = False
                else:
                    b+=1
        return is_filled_up

    def destroy(self):
        if(self.is_empty() == False):
            for b in self.bricks:
                if(b != None):
                    b.remove_from_wall()
                else:
                    print("The brick location is already empty")
        else:
            print("The layer is already empty")

    def build(self):
        for b in self.bricks:
            if(b.is_placed == False):
                b.add_to_wall()

    def fill_feeders(self, feeders):
        # if(self.is_empty() == False):
        for b in self.bricks:
            if(b != None):
                b.find_right_feeder(feeders)

    def to_string(self):
        print "===================== Layer number : {0} =====================".format(self.num)
        for b in self.bricks:
            b.to_string()

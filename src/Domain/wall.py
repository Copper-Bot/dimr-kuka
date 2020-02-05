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

    def __init__(self, feeders, layer_number=3, column_number=4):
        self.layer_number = layer_number
        self.column_number = column_number
        self.bb_number = 0
        self.sb_number = 0
        self.compute_nb_bricks()
        self.layers = [None]*layer_number
        self.is_finished = False
        self.fill()
        self.fill_feeders(feeders)

    def compute_nb_bricks(self):
        nb_even_layers = self.layer_number / 2
        nb_odd_layers = self.layer_number - nb_even_layers
        self.sb_number = nb_odd_layers * 2
        self.bb_number = nb_even_layers * self.column_number + nb_odd_layers * (self.column_number - 1)
    
    def fill(self):
        for l in range(self.layer_number):
            if(l % 2 == 0): #even layer
                self.layers[l] = Layer(l,self.column_number) 
            else: #odd layer
                self.layers[l] = Layer(l,self.column_number + 1)

    def count_placed_bricks(self):
        #count the number of small and big bricks placed in the wall
        #number of small bricks
        sb_number = 0
        #number of big bricks
        bb_number = 0
        for layer in self.layers:
            bricks_nb = layer.count_placed_bricks()
            sb_number += bricks_nb[0]
            bb_number += bricks_nb[1]
        return sb_number, bb_number

    def destroy(self):
        if(self.is_empty() == False):
            for l in self.layers:
                l.destroy()

    def build(self): 
        # if(self.is_empty() == True):
        for l in self.layers:
            # print 'layer in progress : '
            # print(self.layer_in_progress().num)
            l.build()

    def at(self,layer,column):
        if(self.layers[layer] != None):
            return self.layers[layer].bricks[column]
    
    def check_add_brick(self, brick):
        check = True
        if(brick.is_placed == True):
            check = False
        elif(brick.num_layer > 0):
            if(brick.num_layer % 2 == 0): #if the layer is even
                brick_under_1 = self.at(brick.num_layer - 1, brick.num_column)
                brick_under_2 = self.at(brick.num_layer - 1, brick.num_column + 1)
                check = brick_under_1.is_placed and brick_under_2.is_placed
            else: #if layer is odd
                if(brick.num_column == 0):
                    brick_under = self.at(brick.num_layer - 1, brick.num_column)
                    check = brick_under.is_placed
                elif(brick.num_column == self.column_number):
                    brick_under = self.at(brick.num_layer - 1, brick.num_column - 1)
                    check = brick_under.is_placed
                else:
                    brick_under_1 = self.at(brick.num_layer - 1, brick.num_column - 1)
                    brick_under_2 = self.at(brick.num_layer - 1, brick.num_column)
                    check = brick_under_1.is_placed and brick_under_2.is_placed
        return check
            

    def check_remove_brick(self, brick):
        check = True
        if(brick.is_placed == False):
            check = False
        elif(brick.num_layer < layer_number):
            if(brick.num_layer % 2 == 0): #if the layer is even
                brick_under_1 = self.at(brick.num_layer + 1, brick.num_column)
                brick_under_2 = self.at(brick.num_layer + 1, brick.num_column + 1)
                check = not(brick_under_1.is_placed) and not(brick_under_2.is_placed)
            else: #if layer is odd
                if(brick.num_column == 0):
                    brick_under = self.at(brick.num_layer + 1, brick.num_column)
                    check = not(brick_under.is_placed)
                elif(brick.num_column == self.column_number):
                    brick_under = self.at(brick.num_layer + 1, brick.num_column - 1)
                    check = not(brick_under.is_placed)
                else:
                    brick_under_1 = self.at(brick.num_layer + 1, brick.num_column - 1)
                    brick_under_2 = self.at(brick.num_layer + 1, brick.num_column)
                    check = not(brick_under_1.is_placed) and not(brick_under_2.is_placed)
        return check
    
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
        if(self.is_empty() == True):
            print("filling feeders")
            for l in self.layers:
                l.fill_feeders(feeders)
    
    def layer_in_progress(self):
        #return the layer in construction
        l = 0
        layer_found = False
        while(layer_found ==  False and l < len(self.layers)):
            if(self.layers[l].is_filled_up()):
                l += 1
            else:
                layer_found = True
        if(layer_found):
            return self.layers[l]
        else:
            return None
    
    def to_string(self):
        print "####################### Wall begin #######################\n"
        for l in self.layers:
            l.to_string()
        print "####################### Wall end #######################\n\n\n\n"
        

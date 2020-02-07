#!/usr/bin/env python
# -*- coding: utf-8 -*-
from time import sleep

import sys
import numpy as np
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
import tf

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from Domain.wall import Wall
from Domain.feeder import Feeder

class Robot(object):
    def __init__(self, feeders, wall):
        self.init_pose = Pose()
        self.current_pose = Pose()
        self.is_busy = False #robot state to publish on rostopic
        self.feeders = feeders
        self.wall = wall 

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("manipulate_kuka", anonymous=True)
        rospy.loginfo("Beginning motion")

        robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.tfb = tf.TransformBroadcaster() 
        self.tfl = tf.TransformListener()

        rospy.sleep(1) # on attend un peu avant de publier

        rospy.loginfo("adding objects")

        self.add_objects()

    def initialize_effector(self):
        self.is_busy = True
        #TODO define the initial pose of the robot's effector
        self.init_pose.position.x = 0
        self.init_pose.position.y = 0.6
        self.init_pose.position.z = 0.4
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 0
        self.init_pose.orientation.w = 1
        self.move_to(init_pose, False)
        self.is_busy = False

    def update_current_pose(self, pose):
        self.init_pose.position.x = pose.position.x
        self.init_pose.position.y = pose.position.y
        self.init_pose.position.z = pose.position.z
        self.init_pose.orientation.x = pose.orientation.x
        self.init_pose.orientation.y = pose.orientation.y
        self.init_pose.orientation.z = pose.orientation.z
        self.init_pose.orientation.w = pose.orientation.w

    def take_brick_from_wall(self, brick):
        #TODO : check if the brick can be taken

        #place the effector in front of the brick to take in the wall
        target_pose = Pose()
        target_pose.position.x = brick.wall_pose.position.x
        target_pose.position.y = brick.wall_pose.position.y - 0.2
        target_pose.position.z = brick.wall_pose.position.z
        target_pose.orientation.x = brick.wall_pose.orientation.x
        target_pose.orientation.y = brick.wall_pose.orientation.y
        target_pose.orientation.z = brick.wall_pose.orientation.z
        target_pose.orientation.w = brick.wall_pose.orientation.w
        self.move_to(target_pose, False)

        #move the effector forward through the hole in the brick
        self.cartesian_move_to(brick.wall_pose, False)

        #cartesianly lift the brick up of z += 0.1 m (brick.height?)
        target_pose.position.y = self.current_pose.position.y
        target_pose.position.z = self.current_pose.position.z + brick.height
        self.cartesian_move_to(target_pose, True)

        #cartesianly move the brick backward of y -= 0.2
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y - 0.2
        target_pose.position.z = self.current_pose.position.z
        target_pose.orientation.x = self.current_pose.orientation.x
        target_pose.orientation.y = self.current_pose.orientation.y
        target_pose.orientation.z = self.current_pose.orientation.z
        target_pose.orientation.w = self.current_pose.orientation.w
        self.cartesian_move_to(target_pose, True)

        #(to skip?) cartesianly move the brick down of z -= (num_layer + 1)*brick.height

    def take_brick_from_feeder(self, brick):
        #place the end-effector in front of the brick to take
        target_pose = Pose()
        target_pose.position.x = brick.feeder.pose.position.x
        target_pose.position.y = brick.feeder.pose.position.y + 0.2
        target_pose.position.z = brick.feeder.pose.position.z
        target_pose.orientation.x = brick.feeder.pose.orientation.x
        target_pose.orientation.y = brick.feeder.pose.orientation.y
        target_pose.orientation.z = brick.feeder.pose.orientation.z
        target_pose.orientation.w = brick.feeder.pose.orientation.w
        self.move_to(target_pose, False)

        #move the effector forward through the hole in the brick
        self.cartesian_move_to(brick.feeder.pose, True)

        #cartesianly lift the brick up of z += 0.01 m (brick.height?)
        target_pose.position.y = self.current_pose.position.y
        target_pose.position.z = self.current_pose.position.z + 0.01
        self.cartesian_move_to(target_pose, True)
        
        #cartesianly move the brick backward of y += 0.2
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y + 0.2
        target_pose.position.z = self.current_pose.position.z
        target_pose.orientation.x = self.current_pose.orientation.x
        target_pose.orientation.y = self.current_pose.orientation.y
        target_pose.orientation.z = self.current_pose.orientation.z
        target_pose.orientation.w = self.current_pose.orientation.w
        self.cartesian_move_to(target_pose, True)


    def place_brick_in_wall(self, brick):
        #cartesianly move the brick forward of y += 0.2
        target_pose = Pose()
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y + 0.2
        target_pose.position.z = self.current_pose.position.z
        target_pose.orientation.x = self.current_pose.orientation.x
        target_pose.orientation.y = self.current_pose.orientation.y
        target_pose.orientation.z = self.current_pose.orientation.z
        target_pose.orientation.w = self.current_pose.orientation.w
        self.cartesian_move_to(target_pose)

        #cartesianly move the brick down of z -= brick.height
        self.cartesian_move_to(brick.wall_pose)
        
        #move the brick down of 0.01m
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y
        target_pose.position.z = self.current_pose.position.z - 0.01
        target_pose.orientation.x = self.current_pose.orientation.x
        target_pose.orientation.y = self.current_pose.orientation.y
        target_pose.orientation.z = self.current_pose.orientation.z
        target_pose.orientation.w = self.current_pose.orientation.w
        self.cartesian_move_to(target_pose)

        #retract the end-effector
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y - 0.2
        target_pose.position.z = self.current_pose.position.z 
        target_pose.orientation.x = self.current_pose.orientation.x
        target_pose.orientation.y = self.current_pose.orientation.y
        target_pose.orientation.z = self.current_pose.orientation.z
        target_pose.orientation.w = self.current_pose.orientation.w
        self.cartesian_move_to(target_pose)
        

    def place_brick_in_feeder(self, brick):
        #cartesianly move the brick backward of y -= 0.2
        target_pose = Pose()
        target_pose.position.x = self.current_pose.position.x
        target_pose.position.y = self.current_pose.position.y - 0.2
        target_pose.position.z = self.current_pose.position.z
        target_pose.orientation.x = self.current_pose.orientation.x
        target_pose.orientation.y = self.current_pose.orientation.y
        target_pose.orientation.z = self.current_pose.orientation.z
        target_pose.orientation.w = self.current_pose.orientation.w
        self.cartesian_move_to(target_pose)

        #cartesianly move the brick down of z -= brick.feeder.brick_count*brick.height
        self.cartesian_move_to(brick.feeder.pose)

    def cartesian_move_to(self, pose, brick_is_carried):
        #constraint : be careful with the effector's orientation : the brick must not fall
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base"
        target_pose.pose.orientation.w = pose.orientation.w
        target_pose.pose.orientation.x = pose.orientation.x
        target_pose.pose.orientation.y = pose.orientation.y
        target_pose.pose.orientation.z = pose.orientation.z
        target_pose.pose.position.x = pose.position.x
        target_pose.pose.position.y = pose.position.y
        target_pose.pose.position.z = pose.position.z

        #starting and ending points of the path
        waypoints = []
        waypoints.append(self.current_pose)
        waypoints.append(target_pose)

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.05,        # step
                                           5)           # jump_threshold
        print "fraction:", fraction
        # Note: We are just planning, not asking move_group to actually move the robot yet
        #we move the robot :
        self.move_group.execute(plan, wait=True)

        self.update_current_pose(pose)

    def move_to(self, pose, brick_is_carried):
        #constraint : be careful with the effector's orientation : the brick must not fall
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base"
        target_pose.pose.orientation.w = pose.orientation.w
        target_pose.pose.orientation.x = pose.orientation.x
        target_pose.pose.orientation.y = pose.orientation.y
        target_pose.pose.orientation.z = pose.orientation.z
        target_pose.pose.position.x = pose.position.x
        target_pose.pose.position.y = pose.position.y
        target_pose.pose.position.z = pose.position.z

        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=True)
        self.move_group.execute(plan, wait=True) #useless ?
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        self.update_current_pose(target_pose)

    def move_brick_to(self, layer, column):
        brick = self.wall.at(layer, column)
        #2 cases : brick is in the feeder ou brick is in the wall
        self.is_busy = True
        if(brick.is_placed): #the brick is in the wall

            self.take_brick_from_wall(brick)

            #move the brick to target_pose = brick.feeder.pose with target_pose.position.z = brick.feeder.pose.position.z + brick.feeder.brick_count*brick.height
            target_pose = Pose()
            target_pose.position.x = brick.feeder.pose.position.x
            target_pose.position.y = brick.feeder.pose.position.y + 0.2
            target_pose.position.z = brick.feeder.pose.position.z + brick.feeder.brick_count*brick.height
            target_pose.orientation.x = brick.feeder.pose.orientation.x
            target_pose.orientation.y = brick.feeder.pose.orientation.y
            target_pose.orientation.z = brick.feeder.pose.orientation.z
            target_pose.orientation.w = brick.feeder.pose.orientation.w
            self.move_to(self,target_pose, True)

            self.place_brick_in_feeder(brick)

            #update the wall object
            brick.remove_from_wall()
        else:
            self.take_brick_from_feeder(brick.wall_pose)

            #move the brick to its future location in the wall
            target_pose = Pose()
            target_pose.position.x = brick.wall_pose.position.x
            target_pose.position.y = brick.wall_pose.position.y - 0.2
            target_pose.position.z = brick.wall_pose.position.z + brick.height
            target_pose.orientation.x = brick.wall_pose.orientation.x
            target_pose.orientation.y = brick.wall_pose.orientation.y
            target_pose.orientation.z = brick.wall_pose.orientation.z
            target_pose.orientation.w = brick.wall_pose.orientation.w
            self.move_to(self,target_pose, True)

            self.place_brick_in_wall(brick)

            #update the wall object
            brick.add_to_wall()

        self.is_busy = False 
    
    def add_objects(self):

        #Ground in base frame
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "base"
        ground_pose.pose.orientation.w = 1.0
        ground_pose.pose.orientation.x = 0.0
        ground_pose.pose.orientation.y = 0.0
        ground_pose.pose.orientation.z = 0.0
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = 0.0
        ground_name = "ground"
        self.scene.add_box(ground_name, ground_pose, size=(4, 4, 0))

        #Feeders in base frame
        for f in self.feeders:
            feeder_pose = PoseStamped()
            feeder_pose.header.frame_id = "base"
            feeder_pose.pose.orientation.x = f.pose.orientation.x
            feeder_pose.pose.orientation.y = f.pose.orientation.y
            feeder_pose.pose.orientation.z = f.pose.orientation.z
            feeder_pose.pose.orientation.w = f.pose.orientation.w
            feeder_pose.pose.position.x = f.pose.position.x
            feeder_pose.pose.position.y = f.pose.position.y
            feeder_pose.pose.position.z = f.pose.position.z + f.height/2
            feeder_name = "feeder "+str(f.id)
            self.scene.add_box(feeder_name, feeder_pose, size=(f.width, f.depth, f.height))
    
    def add_brick_to_wall(self, brick):
        #Brick in base frame
        brick_pose = PoseStamped()
        brick_pose.header.frame_id = "base"
        brick_pose.pose.orientation.w = brick.pose.orientation.w
        brick_pose.pose.orientation.x = brick.pose.orientation.x 
        brick_pose.pose.orientation.y = brick.pose.orientation.y
        brick_pose.pose.orientation.z = brick.pose.orientation.z
        brick_pose.pose.position.x = brick.pose.position.x
        brick_pose.pose.position.y = brick.pose.position.y
        brick_pose.pose.position.z = brick.pose.position.z
        brick_name = "brick"
        self.scene.add_box(brick_name, brick_pose, size=(brick.width, brick.depth, brick.height))
        

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Copyright 2019:
        Laetitia Lerandy
        Alban Chauvel
        Estelle Arricau

        Projet robotique autonome 2020 DIMR KUKA
        ENSC - ENSEIRB MATMECA 3eme annee option robotique
        code de controle du robot
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
from tf import TransformListener
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from dimr_kuka.msg import DimrControl
from Domain.brick import Type, fill_feeders
from Domain.feeder import Feeder


class Kuka():
    def __init__(self):
        self.is_busy = False #robot state to publish
        self.feeders = fill_feeders()
        for f in self.feeders:
            f.to_string()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("dimr_kuka", anonymous=True)
        rospy.loginfo("Beginning with Kuka")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.eef_link = self.move_group.get_end_effector_link()
        #self.tfb = tf.TransformBroadcaster()
        #self.tfl = tf.TransformListener()
        sub = rospy.Subscriber("kuka_bridge", DimrControl, self.callback_dimrcontrol_message)
        sub = rospy.Subscriber("kuka_destroy", DimrControl, self.callback_dimr_destroy)
        rospy.loginfo("topic kuka_bridge subscribed and ready to process")
        rospy.loginfo("topic kuka_destroy subscribed and ready to process")
        rospy.loginfo("adding objects")
        rospy.sleep(1)
        self.add_objects()

        rospy.spin()
        rospy.loginfo("Stopped manipulation")

    def move_joints(self, joint_goal):

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()

            # Exit
            rospy.loginfo("Stopped manipulation")

    def add_brick_to_feeder(self, pose, brick_type, layer, column):
        #===============remove object=================
        #Brick in base frame
        brick_pose = PoseStamped()
        brick_pose.header.frame_id = "base"
        brick_pose.pose.orientation.w = pose.orientation.w
        brick_pose.pose.orientation.x = pose.orientation.x
        brick_pose.pose.orientation.y = pose.orientation.y
        brick_pose.pose.orientation.z = pose.orientation.z
        brick_pose.pose.position.x = pose.position.x
        brick_pose.pose.position.y = pose.position.y
        brick_pose.pose.position.z = pose.position.z
        brick_name = "brick"+str(layer)+str(column)
        if(brick_type == Type.small.name):
            self.scene.add_box(brick_name, brick_pose, size=(Type.small.value,0.1, 0.1))
        else:
            self.scene.add_box(brick_name, brick_pose, size=(Type.big.value,0.1, 0.1))
        
        if(rospy.get_param(/kuka_destroy/finish)):
            self.add_objects()

    # def add_brick(self, pose, brick_type, brick_name):
    #===============attached object==================
    #     self.scene.remove_attached_object(self.eef_link, name=brick_name)

    def add_brick_to_wall(self, pose, brick_type, layer, column):
        #===============remove object=================
        #Brick in base frame
        brick_pose = PoseStamped()
        brick_pose.header.frame_id = "base"
        brick_pose.pose.orientation.w = pose.orientation.w
        brick_pose.pose.orientation.x = pose.orientation.x
        brick_pose.pose.orientation.y = pose.orientation.y
        brick_pose.pose.orientation.z = pose.orientation.z
        brick_pose.pose.position.x = pose.position.x
        brick_pose.pose.position.y = pose.position.y
        brick_pose.pose.position.z = pose.position.z
        brick_name = "brick"+str(layer)+str(column)
        if(brick_type == Type.small.name):
            self.scene.add_box(brick_name, brick_pose, size=(0.1,Type.small.value, 0.1))
        else:
            self.scene.add_box(brick_name, brick_pose, size=(0.1,Type.big.value, 0.1))

    def remove_brick_from_feeder(self, feeder_pose):
        brick_name = ""
        for f in self.feeders:
            if(f.pose.position.x == feeder_pose.position.x):
                brick_name = "brickf"+str(f.id)+str(f.brick_count-1)

                #===============remove object=================
                self.scene.remove_world_object(brick_name)
                f.remove_brick()

                #===============attached object==================
                # grasping_group = 'manipulator'
                # touch_links = self.robot.get_link_names(group=grasping_group)
                # self.scene.attach_box(self.eef_link, brick_name, touch_links=touch_links)

    def remove_brick_from_wall(self, wall_pose, layer, column):
        brick_name = "brick"+str(layer)+str(column)
        self.scene.remove_world_object(brick_name)

    def callback_dimr_destroy(self,data):
        rospy.loginfo("Message DimrControl has been received.")
        wall_joint_goal = [-6.921975813409511e-05, -1.1366995362220236, 2.509439093453135, 3.1414148918054776, 1.3233576826140818, -3.141619741099784]
        self.move_joints(wall_joint_goal)
        self.move_to(data.brick_pose)
        self.move_joints(wall_joint_goal)
        rospy.sleep(1)
        self.remove_brick_from_wall(data.brick_pose, data.layer, data.column)

        feeder_joint_goal = [1.6427763755008984, -0.3371494753045343, 1.9127279693433106, -0.14040367490179959, -1.376002223374912, 0.10153875293155995]
        self.move_joints(feeder_joint_goal)
        feeder_pose = Pose()
        feeder_pose.position.x = data.feeder_pose.position.x
        feeder_pose.position.y = data.feeder_pose.position.y
        feeder_pose.position.z = data.feeder_pose.position.z
        feeder_pose.orientation.x = 0 #data.feeder_pose.orientation.x
        feeder_pose.orientation.y = 0 #data.feeder_pose.orientation.y
        feeder_pose.orientation.z = 1 #data.feeder_pose.orientation.z
        feeder_pose.orientation.w = 0 #data.feeder_pose.orientation.w
        self.move_to(feeder_pose)
        rospy.sleep(1)
        self.add_brick_to_feeder(feeder_pose, data.brick_type, data.layer, data.column)
        self.move_joints(feeder_joint_goal)
        rospy.loginfo("finish")
        rospy.set_param("/kuka_destroy/busy", False)

    def callback_dimrcontrol_message(self,data):
        rospy.loginfo("Message DimrControl has been received.")

        #intermediate joint pose before taking a brick in a feeder
        # feeder_joint_goal = [1.5620766269453479, -1.7125574664290197, 2.142221452314385, -0.06575124785831427, 1.1193458496424917, 1.5906244190148584]
        feeder_joint_goal = [1.6427763755008984, -0.3371494753045343, 1.9127279693433106, -0.14040367490179959, -1.376002223374912, 0.10153875293155995]
        self.move_joints(feeder_joint_goal)
        feeder_pose = Pose()
        feeder_pose.position.x = data.feeder_pose.position.x
        feeder_pose.position.y = data.feeder_pose.position.y + 0.2
        feeder_pose.position.z = data.feeder_pose.position.z
        feeder_pose.orientation.x = 0.5 
        feeder_pose.orientation.y = 0.5 
        feeder_pose.orientation.z = -0.5 
        feeder_pose.orientation.w = 0.5 
        # self.move_to(feeder_pose)
        self.cartesian_move_to(feeder_pose)

        feeder_pose = Pose()
        feeder_pose.position.x = data.feeder_pose.position.x
        feeder_pose.position.y = data.feeder_pose.position.y + 0.05
        feeder_pose.position.z = data.feeder_pose.position.z 
        feeder_pose.orientation.x = 0.5 
        feeder_pose.orientation.y = 0.5 
        feeder_pose.orientation.z = -0.5 
        feeder_pose.orientation.w = 0.5 
        self.cartesian_move_to(feeder_pose)

        feeder_pose = Pose()
        feeder_pose.position.x = data.feeder_pose.position.x
        feeder_pose.position.y = data.feeder_pose.position.y + 0.05
        feeder_pose.position.z = data.feeder_pose.position.z + 0.01
        feeder_pose.orientation.x = 0.5 
        feeder_pose.orientation.y = 0.5 
        feeder_pose.orientation.z = -0.5 
        feeder_pose.orientation.w = 0.5 
        self.cartesian_move_to(feeder_pose)
        rospy.sleep(1)
        self.remove_brick_from_feeder(data.feeder_pose)
        feeder_pose = Pose()
        feeder_pose.position.x = data.feeder_pose.position.x
        feeder_pose.position.y = data.feeder_pose.position.y + 0.2
        feeder_pose.position.z = data.feeder_pose.position.z + 0.01
        feeder_pose.orientation.x = 0.5 
        feeder_pose.orientation.y = 0.5 
        feeder_pose.orientation.z = -0.5 
        feeder_pose.orientation.w = 0.5 
        self.cartesian_move_to(feeder_pose)



        # wall_joint_goal = [-6.921975813409511e-05, -1.1366995362220236, 2.509439093453135, 3.1414148918054776, 1.3233576826140818, -3.141619741099784]
        # self.move_joints(wall_joint_goal)
        # self.move_to(data.brick_pose)
        wall_pose = Pose()
        wall_pose.position.x = data.brick_pose.position.x - 0.2
        wall_pose.position.y = data.brick_pose.position.y
        wall_pose.position.z = data.brick_pose.position.z #+ 0.1
        wall_pose.orientation.x = 0.0
        wall_pose.orientation.y = 0.707
        wall_pose.orientation.z = 0.0
        wall_pose.orientation.w = 0.707
        self.move_to(wall_pose)
        wall_pose.position.x = data.brick_pose.position.x
        self.cartesian_move_to(wall_pose)
        # wall_pose.position.z = data.brick_pose.position.z
        # self.cartesian_move_to(wall_pose)
        wall_pose.position.x = data.brick_pose.position.x - 0.2
        self.cartesian_move_to(wall_pose)
        rospy.sleep(1)
        #===============remove object=================
        self.add_brick_to_wall(data.brick_pose,data.brick_type,data.layer,data.column)
        # rospy.sleep(2)
        #===============attached object==================
        # if(brick_name != ""):
        #     self.add_brick(data.brick_pose,data.brick_type,brick_name)
        # self.move_joints(wall_joint_goal)
        rospy.loginfo("finish")
        rospy.set_param("/kuka/busy", False)

    def cartesian_move_to(self, target_pose):
        #constraint : be careful with the effector's orientation : the brick must not fall
        waypoints = []
        init_pose = self.move_group.get_current_pose().pose
        # waypoints.append(init_pose)
        waypoints.append(target_pose)

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # step
                                           15)           # jump_threshold
        print "fraction:", fraction
        # Note: We are just planning, not asking move_group to actually move the robot yet
        #we move the robot :
        self.move_group.execute(plan, wait=True)

    def move_to(self, target_pose):
        #constraint : be careful with the effector's orientation : the brick must not fall

        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.go(wait=True)
        # self.move_group.execute(plan, wait=True) #useless ?
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

    def add_objects(self):
        #Reset the scene after the destruction of the wall
        rospy.sleep(1)
        self.scene.remove_world_object()
        rospy.sleep(1)

        #Ground in base frame
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "base"
        ground_pose.pose.orientation.w = 1.0
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z =  -0.01
        ground_name = "ground"
        self.scene.add_box(ground_name, ground_pose, size=(4, 4, 0))

        #Feeders in base frame
        for f in self.feeders:
            k = 0
            for b in f.bricks:
                brick_pose = PoseStamped()
                brick_pose.header.frame_id = "base"
                brick_pose.pose.orientation.w = f.pose.orientation.w
                brick_pose.pose.position.x = f.pose.position.x
                brick_pose.pose.position.y = f.pose.position.y
                brick_pose.pose.position.z = f.pose.position.z + k*b.height
                brick_name = "brickf"+str(f.id)+str(k)
                self.scene.add_box(brick_name, brick_pose, size=(f.width,f.depth,b.height))
                k+=1

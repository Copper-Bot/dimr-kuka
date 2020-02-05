#!/usr/bin/env python
# -*- coding: utf-8 -*-
# from tf import TransformListener
from time import sleep
# import rospy
# from ros4pro.simulation.gripper import Gripper
# from ros4pro.simulation.camera import Camera
# from ros4pro.transformations import multiply_transform, inverse_transform

import sys
import numpy as np
import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
# import tf

from math import pi
from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
from Domain.wall import Wall
from Domain.feeder import Feeder

class Robot(object):
    def __init__(self, wall):
        self.init_pose = Pose()
        self.current_pose = Pose()

    def initialize_effector(self):
        #TODO define the initial pose of the robot's effector
        self.init_pose.position.x = 0
        self.init_pose.position.y = 0.6
        self.init_pose.position.z = 0.4
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 0
        self.init_pose.orientation.w = 1
        self.update_current_pose(self.init_pose)

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
        self.cartesian_move_to(brick.feeder.pose)

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
        self.update_current_pose(pose)

    def move_to(self, pose, brick_is_carried):
        #constraint : be careful with the effector's orientation : the brick must not fall
        self.update_current_pose(pose)

    def move_brick_to(self, brick):
        #2 cases : brick is in the feeder ou brick is in the wall
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
        else:
            self.take_brick_from_feeder(brick)

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
    
    def update_environment():
        #TODO : update class métiers



# class MoveSawyer(object):
#     def __init__(self):
#         super(MoveSawyer, self).__init__()
        
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node("manipulate_sawyer", anonymous=True)

#         self.scene = moveit_commander.PlanningSceneInterface()
#         self.move_group = moveit_commander.MoveGroupCommander("right_arm")
#         self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#                                                        moveit_msgs.msg.DisplayTrajectory,
#                                                        queue_size=20)
#         self.tfb = tf.TransformBroadcaster() 
#         self.tfl = tf.TransformListener()
#         self.camera = Camera()
#         self.gripper = Gripper()  
#         rospy.sleep(1)

#     def display_trajectory(self, plan):

#         self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#         self.display_trajectory.trajectory_start = robot.get_current_state()
#         self.display_trajectory.trajectory.append(plan)
#         # Publish
#         display_trajectory_publisher.publish(display_trajectory)

#         ## END_SUB_TUTORIAL

#     def moveRobotJoint(self, joint_goal):
 
#         # The go command can be called with joint values, poses, or without any
#         # parameters if you have already set the pose or joint target for the group
#         self.move_group.go(joint_goal, wait=True)

#         # Calling ``stop()`` ensures that there is no residual movement
#         self.move_group.stop()

#         # Exit
#         rospy.loginfo("Stopped manipulation")


#     def moveRobotCartesian(self, pose_goal):
        
#         self.move_group.set_pose_target(pose_goal)

#         plan = self.move_group.go(wait=True)
#         # Calling `stop()` ensures that there is no residual movement
#         self.move_group.stop()
#         # It is always good to clear your targets after planning with poses.
#         # Note: there is no equivalent function for clear_joint_value_targets()
#         self.move_group.clear_pose_targets()


#     def z_translation(self, init_pose, amplitude, scale=1):

#         ## BEGIN_SUB_TUTORIAL plan_cartesian_path
#         ##
#         ## Cartesian Paths
#         ## ^^^^^^^^^^^^^^^
#         ## You can plan a Cartesian path directly by specifying a list of waypoints
#         ## for the end-effector to go through. If executing  interactively in a
#         ## Python shell, set scale = 1.0.
#         ##
#         waypoints = []

#         # First move up/down (cf scale) (z)
#         #wpose = self.move_group.get_current_pose().pose
#         waypoints.append(init_pose.pose)

#         # first orient gripper and move forward (+x)
#         wpose = Pose()
#         # wpose.orientation.w = 1.0
#         # wpose.orientation.x = 0.0
#         # wpose.orientation.y = 0.0
#         # wpose.orientation.z = 0.0
#         wpose.position.x = waypoints[0].position.x
#         wpose.position.y = waypoints[0].position.y
#         wpose.position.z = waypoints[0].position.z + scale* amplitude
#         waypoints.append(copy.deepcopy(wpose))
        
#         # self.tfb.sendTransform(wpose.position,wpose.orientation,rospy.Time.now(),'final','cube')


#         '''#tfl = tf.TransformListener()
#         #pose = tfl.lookupTransform('base','cube',rospy.Time(0))
#         #wpose = multiply_transform(pose, wpose); #basePgripper = baseTcube * cubePgripper
#         print "wpose ",wpose
#         waypoints.append(copy.deepcopy(wpose.pose))
#         # end move up/down (cf scale) (z)
#         wpose.pose.orientation.x = 0.
#         wpose.pose.orientation.y = 0.
#         wpose.pose.orientation.z = 0.
#         wpose.pose.orientation.w = 1
#         wpose.pose.position.z += scale * amplitude  
#         waypoints.append(copy.deepcopy(wpose.pose))'''
        
#         '''waypoints_z = np.linspace(wpose.position.z, wpose.position.z + scale * amplitude, 50)  
#         for z in waypoints_z:
#             wpose.position.z = z
#             waypoints.append(copy.deepcopy(wpose))'''

#         # We want the Cartesian path to be interpolated at a resolution of 1 cm
#         # which is why we will specify 0.01 as the eef_step in Cartesian
#         # translation.  We will disable the jump threshold by setting it to 0.0,
#         # ignoring the check for infeasible jumps in joint space, which is sufficient
#         # for this tutorial.
#         (plan, fraction) = self.move_group.compute_cartesian_path(
#                                            waypoints,   # waypoints to follow
#                                            0.01,        # step
#                                            5)           # jump_threshold
#         print "fraction:", fraction
#         # Note: We are just planning, not asking move_group to actually move the robot yet:
#         return plan, fraction

#     def execute_plan(self, plan):

#         ## BEGIN_SUB_TUTORIAL execute_plan
#         ##
#         ## Executing a Plan
#         ## ^^^^^^^^^^^^^^^^
#         ## Use execute if you would like the robot to follow
#         ## the plan that has already been computed:
#         self.move_group.execute(plan, wait=True)



#     def addFeederAndFloor(self):
#         """Add the floor and the the feeder box to the scene in order to avoid collisions"""

#         floor_pose = PoseStamped()
#         floor_pose.header.frame_id = "base"
#         floor_pose.pose.orientation.x = 0.
#         floor_pose.pose.orientation.y = 0.
#         floor_pose.pose.orientation.z = 0.
#         floor_pose.pose.orientation.w = 1.0
#         floor_pose.pose.position.x = 0.
#         floor_pose.pose.position.y = 0.
#         floor_pose.pose.position.z = -0.125
#         self.scene.add_box("floor", floor_pose,  size=(3, 3, 0.01))

#         feeder_pose = PoseStamped()
#         feeder_pose.header.frame_id = "base"
#         feeder_pose.pose.orientation.x = 0.
#         feeder_pose.pose.orientation.y = 0.
#         feeder_pose.pose.orientation.z = 0.
#         feeder_pose.pose.orientation.w = 1.0
#         feeder_pose.pose.position.x = 0.25
#         feeder_pose.pose.position.y = 0.56
#         feeder_pose.pose.position.z = 0.06
#         self.scene.add_box("feeder", feeder_pose, size=(0.34, 0.32, 0.37))

#         pedestal_pose = PoseStamped()
#         pedestal_pose.header.frame_id = "base"
#         pedestal_pose.pose.orientation.x = 0.
#         pedestal_pose.pose.orientation.y = 0.
#         pedestal_pose.pose.orientation.z = 0.
#         pedestal_pose.pose.orientation.w = 1.0
#         pedestal_pose.pose.position.x = -0.09
#         pedestal_pose.pose.position.y = 0.
#         pedestal_pose.pose.position.z = -0.0625
#         self.scene.add_box("pedestal", pedestal_pose, size=(0.64, 0.8, 0.125))

#         # pose = PoseStamped()
#         # pose.header.frame_id = "base"
#         # pose.pose.orientation.x = 0.
#         # pose.pose.orientation.y = 0.
#         # pose.pose.orientation.z = 0.
#         # pose.pose.orientation.w = 1.0
#         # pose.pose.position.x = 0.32
#         # pose.pose.position.y = 0.52
#         # pose.pose.position.z = 0.37 -0.125 + 0.053/2
#         # self.scene.add_box("cube_orange", pose, size=(0.053, 0.053, 0.053))


    
#     def addCubeFrame(self):


#         rospy.sleep(1)
#         self.tfb.sendTransform([0.32,0.52,0.37 -0.125 + 0.053/2],[1,0,0,0],rospy.Time.now(),'cube','base')
        
       

# if __name__ == "__main__":
   
#     MS = MoveSawyer()
#     MS.addCubeFrame()
#     MS.addFeederAndFloor()
#     rospy.sleep(1)
#     MS.gripper.open()
#     rospy.sleep(1)
#     # MS.moveRobot([-pi/2, pi/2, -pi/2, 0, 0 , 0, 0])
   
#     print  MS.scene.get_known_object_names()


    
#     # The approach:
#     amplitude = 0.18 + 0.053/2.0
#     approach = PoseStamped()
#     approach.header.frame_id = "cube"
#     approach.pose.position.z -= amplitude
#     # approach.pose.orientation.w = 1
#     while True:
#         MS.moveRobotCartesian(approach)
#         rospy.sleep(1)
#         # # liste = pose_to_list(approach.pose)
#         # # MS.tfb.sendTransform(liste[0],liste[1],rospy.Time.now(),'init','cube')
#         #
#         # (cartesian_plan, fraction) = MS.z_translation(approach, amplitude, scale=1)
#         # MS.execute_plan(cartesian_plan)
#         # rospy.sleep(1)
        
#         rospy.sleep(10)
#         transorm = self.tfl.lookupTransform('base', 'right_gripper_tip', rospy.Time(0))


#         translat = geometry_msgs.msg.Pose()
#         # wpose5 = multiply_transform(pose_goal, transorm)
#         translat.position.x = transorm[0][0]
#         translat.position.y = transorm[0][1]
#         translat.position.z= transorm[0][2]
#         translat.orientation.x= transorm[1][0]
#         translat.orientation.y= transorm[1][1]
#         translat.orientation.z= transorm[1][2]
#         translat.orientation.w= transorm[1][3]

#         # pose_goal = geometry_msgs.msg.Pose()
#         waypoints5 = []

#         waypoints5.append(copy.deepcopy(translat))
#         #wpose5.orientation.x = 1.0
#         # wpose5.position.x = approach.pose.position.x
#         # wpose5.position.y = approach.pose.position.y
#         translat.position.z -= 0.18 + 0.053/2.0

#         waypoints5.append(copy.deepcopy(translat))

#         (plan5, fraction5) = MS.move_group.compute_cartesian_path(
#             waypoints5,  # waypoints to follow
#             0.01,  # eef_step
#             0.0)  # jump_threshold

#         MS.move_group.execute(plan5, wait=True)
#         MS.move_group.clear_pose_targets()
#         rospy.sleep(1)

#         MS.gripper.close()
#         rospy.sleep(1)

#         MS.moveRobotCartesian(approach)
#         rospy.sleep(1)

#         pose = PoseStamped()
#         pose.header.frame_id = "base"
#         pose.pose.position.x = 0.5
#         pose.pose.position.y = 0
#         pose.pose.position.z = 0.1
#         pose.pose.orientation.x = 0.707
#         pose.pose.orientation.y = 0.707
#         pose.pose.orientation.z = 0
#         pose.pose.orientation.w = 0
#         # pose.pose.orientation.w = 1
#         MS.moveRobotCartesian(pose)
#         rospy.sleep(1)

#         MS.gripper.open()
#         rospy.sleep(1)

#         pose.pose.position.z += 0.1
#         MS.moveRobotCartesian(pose)
#         rospy.sleep(1)

#         camera.shoot()
#         img = rospy.wait_for_message("/io/internal_camera/right_hand_camera/image_rect", Image)
#         print "Requesting (label, x, y)"
#         (label,x_pixels,y_pixels) = vision_client(img)
#         #coordinates in the camera frame
#         x_cam = (x_pixels - 752/2)*0.31/752
#         y_cam = (y_pixels - 480/2)*0.195/480
#         z_cam = 0.2
#         transform = self.tfl.lookupTransform('base', 'right_hand_camera', rospy.Time(0))

#         cube = geometry_msgs.msg.Pose()
#         cube.position.x = transform[0][0] + x_cam
#         cube.position.y = transform[0][1] + y_cam
#         cube.position.z= transform[0][2] + z_cam
#         cube.orientation.x= transform[1][0]
#         cube.orientation.y= transform[1][1]
#         cube.orientation.z= transform[1][2]
#         cube.orientation.w= transform[1][3]

# def vision_client(img):
#     rospy.wait_for_service('vision_infer')
#     try:
#         handle_process = rospy.ServiceProxy('vision_infer_server', VisionInfer)
#         (label,x,y) = handle_process(img)
#         return (label,x,y)
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e
    

# # def callback(data):
# #     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
# # def listener():

# #     # In ROS, nodes are uniquely named. If two nodes with the same
# #     # name are launched, the previous one is kicked off. The
# #     # anonymous=True flag means that rospy will choose a unique
# #     # name for our 'listener' node so that multiple listeners can
# #     # run simultaneously.
# #     rospy.init_node('listener', anonymous=True)

# #     rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, callback)

# #     # spin() simply keeps python from exiting until this node is stopped
# #     rospy.spin()

#     '''tfl = tf.TransformListener()
#     pose = tfl.lookupTransform(‘base’, ‘cube’, rospy.Time(0))
#     print "pose", pose'''
    


# '''

#     camera = Camera()
#     gripper = Gripper()

#     # Command gripper open() or close()
#     for _ in range(5):
#        gripper.open()
#        sleep(1)
#        gripper.close()
#        sleep(1)


#     # Take a picture.
#     # The image will be published on topic /io/internal_camera/right_hand_camera/image_rect. It can be retrieved with a subscriber
#     #camera.shoot()

# '''

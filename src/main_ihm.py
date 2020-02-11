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

from App.ihm import Ihm
from Domain.wall import Wall
from Domain.brick import Type

if __name__ == '__main__':
    try:
        app = Ihm()
        app.title("DIMR KUKA")
        app.geometry('800x500')
        app.mainloop()
    except KeyboardInterrupt:
        print('Killed by user')
        sys.exit(0)

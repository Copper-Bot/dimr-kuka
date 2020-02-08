#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np
import copy
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from Domain.feeder import Feeder
from Domain.brick import Type
from App.kuka import Kuka

def main():
    taking_pose_b6 = Pose()
    taking_pose_b6.position.x = 0
    taking_pose_b6.position.y = -0.5
    taking_pose_b6.position.z = 0
    taking_pose_b6.orientation.w = 1

    f1 = Feeder(1, 6, Type.big, taking_pose_b6)

    #TODO : input the taking coordinates for the b5 runner
    taking_pose_b5 = Pose()
    taking_pose_b5.position.x = -0.3
    taking_pose_b5.position.y = -0.5
    taking_pose_b5.position.z = 0
    taking_pose_b5.orientation.w = 1

    f2 = Feeder(2, 5, Type.big, taking_pose_b5)

    #TODO : input the taking coordinates for the s2 runner
    taking_pose_s2 = Pose()
    taking_pose_s2.position.x = 0.3
    taking_pose_s2.position.y = -0.5
    taking_pose_s2.position.z = 0
    taking_pose_s2.orientation.w = 1

    f3 = Feeder(3, 2, Type.small, taking_pose_s2)
    feeders = [f1,f2,f3]
    kuka = Kuka(feeders)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Killed by user')
        sys.exit(0)

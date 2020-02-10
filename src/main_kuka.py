#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import numpy as np
import copy
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from App.kuka import Kuka

if __name__ == '__main__':
    try:
        # kuka = Kuka(feeders)
        kuka = Kuka()
    except KeyboardInterrupt:
        print('Killed by user')
        sys.exit(0)

#!/usr/bin/env python
# -*- config utf-8 -*-

import rospy
import sys
import roslib

file_path = roslib.packages.get_pkg_dir("happymimi_teleop") + "/src/"
sys.path.insert(0, file_path)
from base_control import BaseControl

rospy.init_node('test')

bc = BaseControl()
bc.translateDist(2.0, 1.0)

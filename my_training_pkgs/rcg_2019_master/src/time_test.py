#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

rospy.init_node('time_test')
start_time = rospy.get_time()
while not rospy.is_shutdown():
    print(start_time)
    print(rospy.get_time() - start_time)

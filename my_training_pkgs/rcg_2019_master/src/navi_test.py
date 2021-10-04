#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
#Title: オブジェクトをテーブルから棚へカテゴリごとに移動させる
#Author: Kanazawa Yusuke
#Data: 2021/9.6
#Memo
#-------------------------------------------------------------------------------
import rospy
from std_msgs.msg import String
import sys
from happymimi_navigation.srv import NaviLocation, NaviLocationResponse

rospy.init_node('navi_test')

location_list = rospy.ServiceProxy('navi_location_server', NaviLocation)
location_list('table')

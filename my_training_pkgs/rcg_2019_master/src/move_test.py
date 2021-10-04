#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------
#Title: メガローバーを指定した距離移動させるテストノード
#Author: Kanazawa Yusuke
#Data: 2021/9.8
#Memo
#-------------------------------------------------------------------
import rospy
from geometry_msgs.msg import Twist

#vel = 0.2 仮速度
#dist = ? 進む距離を指定する
#time = dist/vel 速度と距離から動貸し続ける時間を計算する(v=m/s)

class MoveTest():
    def __init__(self):
        self.pub = rospy.Publisher('/vmegarover/diff_drive_controller/cmd_vel', Twist, queue_size = 1)

    def execute(self, user_vel, user_dist):
        print('execute')
        vel = Twist()
        vel.linear.x = user_vel
        dist = user_dist

        time = dist/vel.linear.x
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (rospy.get_time() - start_time) <= time:
                self.pub.publish(vel)
                print(rospy.get_time() - start_time)
            else:
                print('break')
                print(rospy.get_time() - start_time)
                break

        rospy.loginfo('success')

if __name__ == '__main__':
    rospy.init_node('move_test')
    mv = MoveTest()
    mv.execute()

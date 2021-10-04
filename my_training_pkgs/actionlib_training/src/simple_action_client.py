#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy

import actionlib
from actionlib_training.msg import TimerAction, TimerGoal, TimerResult

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', TimerAction)
# サーバー側の５つのトピックの起動が完了するのを確認するのがwait_for_server それまではブロック
client.wait_for_server()

# TimerGoal型のゴールを作成し、値を代入したらサーバーにゴールメッセージを送る
goal = TimerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)
client.send_goal(goal)
client.wait_for_result()
print('Time elapsed: %f' %(client.get_result().time_elapsed.to_sec()))

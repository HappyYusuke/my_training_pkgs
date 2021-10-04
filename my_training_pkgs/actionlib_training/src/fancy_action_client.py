#!/usr/bin/env python
# -*-coding: utf-8 -*-
import rospy

import time
import actionlib
from actionlib_training.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback


def feedback_cb(feedback):
    print('[Feedback] Time elapsed: %f' %(feedback.time_elapsed.to_sec()))
    print('[Feedback] Time remaining: %f' %(feedback.time_remaining.to_sec()))


rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', TimerAction)
client.wait_for_server()

goal = TimerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)
#サーバー側での強制終了をテストするには次の行のコメントアウトを外してください
#goal.time_to_wait = rospy.Duration.from_sec(500.0)
client.send_goal(goal, feedback_cb=feedback_cb)

# ゴールの中断をテストするには次の２行のコメントアウトを外してください
#time.sleep(3.0)
#client.cancel_goal()

# get.state関数はゴールの状態を返します
client.wait_for_result()
print('[Result] State: %d' %(client.get_state()))
print('[Result] Status: %s' %(client.get_goal_status_text()))
print('[Result] Time elapsed: %f' %(client.get_result().time_elapsed.to_sec()))
print('[Result] Updates sent: %d' %(client.get_result().updates_sent))

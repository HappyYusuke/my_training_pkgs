#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import actionlib
from ros_start.msg import GoUntilBumperAction
from ros_start.msg import GoUntilBumperResult
from ros_start.msg import GoUntilBumperFeedback

class BumperAction(object):
    def __init__(self):
        self._pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self._sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback, queue_size=1)
        self._max_vel = rospy.get_param('~max_vel', 0.2)

        # アクションサーバーを定義
        self._action_server = actionlib.SimpleActionServer('bumper_action', GoUntilBumperAction, execute_cb=self.go_until_bumper, auto_start=False)
        self._hit_bumper = False

        # 準備ができたらサーバースタート
        self._action_server.start()

    def bumper_callback(self, bumper):
        self._hit_bumper = True

    # アクションサーバーが呼び出されたときの処理内容（アクションサーバーの実態）
    def go_until_bumper(self, goal):
        print(goal.target_vel)
        r = rospy.Rate(10.0)
        zero_vel = Twist()
        while not rospy.is_shutdown():
            #print('while now')
            # 停止司令が出ているかを「is_preempt_requested()」で調べる
            if self._action_server.is_preempt_requested():
                #print('stop')
                self._action_server.is_preempt_requested()
                break
            # バンパーがぶつかったときにbumper_callbackでself._hit_bumperがTrueになる
            elif self._hit_bumper:
                #print('hit_bumper')
                self._pub.publish(zero_vel)
                break
            elif goal.target_vel.linear.x < self._max_vel:
                #print('start')
                self._pub.publish(goal.target_vel)
                feedback = GoUntilBumperFeedback(current_vel=goal.target_vel)
                self._action_server.publish_feedback(feedback)
                r.sleep()
                result = GoUntilBumperResult(bumper_hit=self._hit_bumper)
                self._action_server.set_succeeded(result)
            else:
                #print('what?')
                pass

if __name__ == '__main__':
    rospy.init_node('bumper_action')
    bumper_action = BumperAction()
    rospy.spin()

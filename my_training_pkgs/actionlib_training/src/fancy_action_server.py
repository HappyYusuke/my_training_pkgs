#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy

import time
import actionlib
from actionlib_training.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback


def do_timer(goal):
    start_time = time.time()
    # フィードバックを配信した回数を保存するための変数
    update_count = 0

    # time_to_waitが60病より大きい場合はset_aborted()を呼ぶことでゴールを強制終了する
    # このタイマーを長い時間待つために使ってほしくない
    # set_aborted()の呼び出しにより、クライアントに対して
    # ゴールが強制終了されたことを通知するメッセージが送られる
    if goal.time_to_wait.to_sec() > 60.0:
        result = TimerResult()
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count
        server.set_aborted(result, "Timer aborted due to too-long wait")
        return

    # 一度にスリープするのではなく、ループで少しずつスリープすることで
    # 割り込みの確認やフィードバックを提供したりといった処理ができる
    while (time.time() - start_time) < goal.time_to_wait.to_sec():

        # is_preempt_requestedは割り込みの確認（クライアントがゴールの中断を要求したときにTrueを返す）
        if server.is_preempt_requested():
            result = TimerResult()
            result.time_elapsed = \
                rospy.Duration.from_sec(time.time() - start_time)
            result.updates_sent = update_count
            server.set_preempted(result, "Timer preempted")
            return

        # フィードバックを送る
        feedback = TimerFeedback()
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed
        server.publish_feedback(feedback)
        update_count += 1

        time.sleep(1.0)

        # クライアントに終了を通知する
        result = TimerResult()
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count
        server.set_succeeded(result, "Timer completed successfully")


rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
server.start()
rospy.spin()

# クライアントが早く終わりすぎて通信相手がいないよーーーー

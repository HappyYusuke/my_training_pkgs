#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy

import time
import actionlib
from actionlib_training.msg import TimerAction, TimerGoal, TimerResult

# 引数のgoalはTimerGoal型
def do_timer(goal):
    # time_to_waitフィールドをROSの期間を示す型(Duration)の値から秒に変換する必要がある
    start_time = time.time()
    time.sleep(goal.time_to_wait.to_sec())

    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() -start_time)
    result.updates_sent = 0

    # SimpleActionServerにゴールを達成したことを伝える
    server.set_succeeded(result)

rospy.init_node('timer_action_server')

# 最後のFalseはサーバーの自動起動を無効にするためのFalse
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
# アクションサーバーの生成が終わったら、start()を呼ぶ
server.start()
rospy.spin()



#------------------------------------------
# 自分が定義していない謎メッセージの説明
#------------------------------------------
# rostopic info /timer/goalで出てくる/TimerActionGoalは何か

# rosmsg show TimerActionGoalでみてみると自分の定義していないフィールドがいくつか追加されてる

# これらのフィールドはアクションサーバーとクライアントの間で何が起きているかを管理するために
# 内部的に使われる

# この管理のための情報は、私達のサーバー側のコードが
# ゴールメッセージとして受け取る前に自動的に削除される
# なので、管理のための情報は私達は触る必要がない

# トピック上にはTimerActionGoalメッセージが送られるが、
# 私達が実際に受け取るのは、.action定義ファイルで定義したTimerGoalメッセージそのものだ
# rosmsg show TimerGoal

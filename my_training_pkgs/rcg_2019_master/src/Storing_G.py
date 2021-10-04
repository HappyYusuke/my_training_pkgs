#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-----------------------------------------------------------------
#Title: オブジェクトをテーブルから棚へカテゴリごとに移動させる
#Author: Kanazawa Yusuke
#Data: 2021/9.6
#Memo
#-----------------------------------------------------------------

import rospy
import sys
import roslib
import smach
import smach_ros

file_path = roslib.packages.get_pkg_dir("happymimi_teleop") + "/src/"
sys.path.insert(0, file_path)
from base_control import BaseControl

from std_msgs.msg import String
from happymimi_navigation.srv import NaviLocation, NaviLocationResponse
from happymimi_msgs.srv import StrTrg
from mimi_manipulation_pkg.srv import ManipulateSrv, RecognizeCount

#棚の扉開く
class OpenShelf():
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_PAP'])

        self.arm_changer = rospy.ServiceProxy('/servo/arm', StrTrg)
        self.move_location = rospy.ServiceProxy('navi_location_server', NaviLocation)
        self.endeffector = rospy.Publisher('/servo/endeffector', Bool, queue_size = 1)
        self.bace_con = BaseControl()
    
    def execute(self):
        self.move_location('shelf') #棚の前に移動する
        rospy.sleep(1.0)
        
        self.endefector.publish(True) #エンドエフェクター閉じる
        self.arm_changer('origin') #アームを水平にする（エンドエフェクター:閉）
        rospy.sleep(1.0)
        
        self.base_con.translateDist(0.1, 0.2) #アームが棚に入るまで進む
        self.endeffector.publish(False) #エンドエフェクター閉じる
        rospy.sleep(1.0)
        
        self.base_con.translateDist(0.3, -0.2) #棚がある程度開くまで後退する
        rospy.sleep(1.0)
        
        self.base_con.rotateAngle(45, 0.5) #回転する（棚を開ききるため）
        rospy.sleep(1.0)
        
        self.base_con.translateDist(0.15, 0.2) #棚のドアを押し切って開ききる
        rospy.sleep(1.0)

        self.endeffector.publish(True) #もとの姿勢に戻る
        arm_changer('carry')
        self.base_con.rotateAngle(45, -0.5)

        return 'to_PAP'

#棚のオブジェクトカテゴリ認識
#テーブルに移動
#テーブルから棚にオブジェクトを移動

object_num = 0

class MoveAndPick():
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_PLACE'])

        #物体認識関連をインスタンス化する行（まだないモジュールがある）
        self.base_con = BaseControl()
        self.grab = rospy.ServiceProxy('/manipulation', ManipulateSrv)
        self.recog = rospy.ServiceProxy('/object/recognize', RecognizeCount)
        self.count = rospy.ServiceProxy('/recognize/count', RecognizeCount)

    def execute(self):
        #棚でオブジェクトのカテゴリ認識
        recog = self.recog('any')
        
        if len(recog.data) <= 3:
            object_name = recog.data[0]
            object_num = self.count('object_name')
        else:
            rospy.loginfo('There are no objects')

        #180度回転してテーブル上のオブジェクト認識
        self.base_con.rotateAngle(180, 0.5)
        
        #把持
        result = self.grab('object_name').result
        if result == True:
            return 'to_PLACE'
        else:
            rospy.loginfo('MoveAndPick failed')


class MoveAndPlace():
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_PICK', 'completed'])

        self.arm = rospy.ServiceProxy('/servo/arm', ManipulateSrv)
        self.move_location = rospy.ServiceProxy('navi_location_server', NaviLocation)

    def execute(self):
        self.move_location('shelf')
        rospy.sleep(1.0)
        self.arm('place')

        if object_num != 0:
            return 'to_PICK'
        else:
            return 'completed'

def main():
    rospy.loginfo('Start Storing Groceries')

    # 状態機械を定義
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    #出力結果と遷移先を定義
    with sm_top:
        smach.StateMachine.add('OpenShelf', OpenShelf(), 
                                transitions={'to_PAP':'PickAndPlace'})

        sm_PAP = smach.StateMachine(outcomes=['succeeded'])

        with sm_PAP:
            smach.StateMachine.add('MoveAndPick', MoveAndPick(),
                                    transitions={'to_PLACE':'MoveAndPlace'})

            smach.StateMachine.add('MoveAndPlace', MoveAndPlace(),
                                    transitions={'to_PICK':'MoveAndPick', 'complete':'completed'})

        smach.StateMachine.add('PickAndPlace', sm_PAP,
                                transitions={'succeeded':'Finish_Storing_G'})
    
    # smach_viewerで見えるようにする
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # 状態機会実行
    outcome = sm_top.execute()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Storing_G')
    main()

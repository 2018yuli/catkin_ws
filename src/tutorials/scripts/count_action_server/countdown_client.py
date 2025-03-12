#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import actionlib
from tutorials.msg import CountdownAction, CountdownGoal, CountdownResult

# 反馈回调
def feedback_cb(feedback):
    rospy.loginfo("Countdown at: %d" % feedback.current_num)

def done_cb(status, result):
    # 可以在这里处理完成后的结果
    rospy.loginfo("Done callback: %s" % result.result_msg)

def active_cb():
    rospy.loginfo("Action is active...")

# 初始化客户端节点
rospy.init_node('countdown_client')
client = actionlib.SimpleActionClient('countdown', CountdownAction)
rospy.loginfo("Waiting for action server to start...")

# 等待服务器启动
client.wait_for_server()

# 初始化参数
goal = CountdownGoal()
goal.start_num = 25
rospy.loginfo("Sending countdown goal: %d" % goal.start_num)

# 发送目标，并注册反馈回调
client.send_goal(goal, feedback_cb=feedback_cb, done_cb=done_cb, active_cb=active_cb)
client.wait_for_result()

# 获取结果
result = client.get_result()
rospy.loginfo("Result: %s" % result.result_msg)
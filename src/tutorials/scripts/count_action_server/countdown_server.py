#!/usr/bin/env python2
# -*- coding: utf-8 -*-  # 添加此行

import rospy
import actionlib
from tutorials.msg import CountdownAction, CountdownFeedback, CountdownResult


def execute_cb(goal):
    # 初始化 with: goal.start_num
    feedback = CountdownFeedback()
    result = CountdownResult()
    rospy.loginfo("Starting countdown from %d" % goal.start_num)
    
    # 倒计时循环
    for i in range(goal.start_num, -1, -1):
        # 检测是否取消任务
        if server.is_preempt_requested():
            rospy.loginfo("Countdown preempted")
            server.set_preempted()
            return
        
        # 发送反馈
        feedback.current_num = i
        server.publish_feedback(feedback)  
        
        # 每秒倒计时
        rospy.sleep(1)  
    
    # 完成动作
    result.result_msg = "Countdown completed!"
    server.set_succeeded(result)  # 发送结果
    
# 初始化节点
rospy.init_node('countdown_server')
# autoStart = False
server = actionlib.SimpleActionServer('countdown', CountdownAction, execute_cb, False)
server.start()
rospy.loginfo("Countdown Action Server is ready.")
rospy.spin()
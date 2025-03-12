#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    

def listener():
    rospy.init_node('listener', anonymous=True)
    # 在 python 中 rospy.Subscriber 会启动自己的线程
    rospy.Subscriber("chatter", String, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
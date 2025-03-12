#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

import rospy
import nodelet
from std_msgs.msg import String

class TalkerNodelet(nodelet.Nodelet):
    def __init__(self):
        super(TalkerNodelet, self).__init__()

    def onInit(self):
        rospy.loginfo("TalkerNodelet Initialized")
        self.pub = rospy.Publisher("/chatter2", String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_message)
        
    def publish_message(self, event):
        msg = String()
        msg.data = "Hello from TalkerNodelet"
        rospy.loginfo(msg.data)
        self.pub.publish(msg)
#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

import rospy
import nodelet
from std_msgs.msg import String
class ListenerNodelet(nodelet.Nodelet):
    def __init__(self):
        super(ListenerNodelet, self).__init__()
    def onInit(self):
        rospy.loginfo("ListenerNodelet Initialized")
        self.sub = rospy.Subscriber("/chatter2", String, self.callback)
    def callback(self, msg):
        rospy.loginfo("ListenerNodelet heard: %s", msg.data)
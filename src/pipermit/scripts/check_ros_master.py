#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosnode
import rospy

def check_ros_master():
    """
    检查ROS Master是否已运行，如果未运行则抛出异常。
    """
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

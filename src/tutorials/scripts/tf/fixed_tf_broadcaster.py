#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('tutorials')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')

    # Create a TransformBroadcaster object
    br = tf.TransformBroadcaster()

    # Set loop rate to 10 Hz
    rate = rospy.Rate(10.0)

    try:
        while not rospy.is_shutdown():
            # Broadcast transform (commented out)
            br.sendTransform((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "carrot1", "turtle1")
            rate.sleep()

    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("ROS shutdown requested, terminating node.")

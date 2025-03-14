#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('tutorials')

import rospy
import geometry_msgs.msg
import turtlesim.srv
from turtlesim.msg import Pose


def spawn_turtle(x, y, theta, name):
    """
    Spawns a new turtle at the specified position and angle.
    """
    rospy.wait_for_service('spawn')
    try:
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(x, y, theta, name)
        rospy.loginfo("%s spawned at (%f, %f, %f)" % (name, x, y, theta))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def handle_turtle_pose(msg):
    """
    Handles the pose data of the turtle and logs it.
    """
    rospy.loginfo("Turtle Pose -> X: %f, Y: %f, Theta: %f" % (msg.x, msg.y, msg.theta))


def control_turtle():
    """
    Controls the movement of the turtle by publishing velocity commands.
    """
    # Create a publisher to control the turtle's velocity
    turtle_vel = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    # Set loop rate to 10 Hz (10 times per second)
    rate = rospy.Rate(10.0)

    # Loop until ROS is shut down
    while not rospy.is_shutdown():
        cmd = geometry_msgs.msg.Twist()
        # Set linear velocity along x-axis and angular velocity around z-axis
        cmd.linear.x = 3
        cmd.angular.z = 3
        # Publish the velocity command to move the turtle
        turtle_vel.publish(cmd)
        rate.sleep()


def main():
    """
    Initializes the ROS node, spawns a turtle, subscribes to its pose, and controls its movement.
    """
    rospy.init_node('turtle_control')

    # Spawn a new turtle
    spawn_turtle(5.5, 4.5, 0, 'turtle3')

    # Subscribe to the turtle's pose
    rospy.Subscriber('turtle3/pose', Pose, handle_turtle_pose)

    rospy.loginfo("The turtle3 has been spawned and is now being controlled.")

    # Start controlling the turtle's movement
    control_turtle()


if __name__ == '__main__':
    main()

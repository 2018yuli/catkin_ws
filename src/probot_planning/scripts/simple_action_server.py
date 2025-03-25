#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from std_msgs.msg import Float32MultiArray

# 目标位置
cp1, cp2, cp3, cp4, cp5, cp6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
tp1, tp2, tp3, tp4, tp5, tp6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

# 收到 action 的 goal 后调用的回调函数
def execute_callback(goal):
    global cp1, cp2, cp3, cp4, cp5, cp6
    global tp1, tp2, tp3, tp4, tp5, tp6

    rospy.loginfo("[Action] Received new goal from client.")
    rospy.loginfo("[Action] Joint names: %s", goal.trajectory.joint_names)

    try:
        # 当前值
        cp1 = goal.trajectory.points[0].positions[0]
        cp2 = goal.trajectory.points[0].positions[1]
        cp3 = goal.trajectory.points[0].positions[2]
        cp4 = goal.trajectory.points[0].positions[3]
        cp5 = goal.trajectory.points[0].positions[4]
        cp6 = goal.trajectory.points[0].positions[5]

        # 目标值
        tp1 = goal.trajectory.points[1].positions[0]
        tp2 = goal.trajectory.points[1].positions[1]
        tp3 = goal.trajectory.points[1].positions[2]
        tp4 = goal.trajectory.points[1].positions[3]
        tp5 = goal.trajectory.points[1].positions[4]
        tp6 = goal.trajectory.points[1].positions[5]
    except IndexError as e:
        rospy.logerr("[Action] Goal trajectory format error: %s", str(e))
        server.set_aborted()
        return

    rospy.loginfo("[Action] Current joint values: [%f, %f, %f, %f, %f, %f]", cp1, cp2, cp3, cp4, cp5, cp6)
    rospy.loginfo("[Action] Target joint values: [%f, %f, %f, %f, %f, %f]", tp1, tp2, tp3, tp4, tp5, tp6)

    # 发布反馈
    feedback = FollowJointTrajectoryFeedback()
    feedback.desired.positions = [tp1, tp2, tp3, tp4, tp5, tp6]
    feedback.actual.positions = [cp1, cp2, cp3, cp4, cp5, cp6]
    server.publish_feedback(feedback)
    rospy.loginfo("[Action] Published feedback.")

    # 设置任务成功
    result = FollowJointTrajectoryResult()
    server.set_succeeded(result)
    rospy.loginfo("[Action] Goal succeeded and result sent.")

if __name__ == "__main__":
    rospy.loginfo("[System] Starting trajectory_server node...")
    rospy.init_node('trajectory_server')

    rospy.loginfo("[System] Creating SimpleActionServer...")
    server = actionlib.SimpleActionServer(
        'probot_anno/arm_joint_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
        execute_callback,
        auto_start=False
    )

    rospy.loginfo("[System] Starting Action Server...")
    server.start()

    rospy.loginfo("[System] Action Server is ready and waiting for goals.")
    rospy.spin()

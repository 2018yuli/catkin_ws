#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
import threading
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PiperStatusMsg, PosCmd, PiperEulerPose
from tf.transformations import quaternion_from_euler

class PublishSubscribeManager:
    """
    用于管理该节点的 发布(发布关节、末端姿态等) 和 订阅(关节命令、pos命令、使能命令等) 逻辑。
    """
    def __init__(self, node, param_config):
        self.node = node  # 引用 C_PiperRosNode
        self.param_config = param_config

        # 创建并设置默认 JointState
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0]*7
        self.joint_states.velocity = [0.0]*7
        self.joint_states.effort = [0.0]*7

        # 初始化Publisher
        self.joint_pub = rospy.Publisher('joint_states_single', JointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher('arm_status', PiperStatusMsg, queue_size=1)
        # self.arm_joint_status_rviz = rospy.Publisher('joint_status', JointState, queue_size=1)
        self.end_pose_pub = rospy.Publisher('end_pose', PoseStamped, queue_size=1)
        self.end_pose_euler_pub = rospy.Publisher('end_pose_euler', PiperEulerPose, queue_size=1)

        # 启动订阅线程
        sub_pos_th = threading.Thread(target=self.sub_pos_thread)
        sub_pos_th.daemon = True
        sub_pos_th.start()

        sub_joint_th = threading.Thread(target=self.sub_joint_thread)
        sub_joint_th.daemon = True
        sub_joint_th.start()

        sub_enable_th = threading.Thread(target=self.sub_enable_thread)
        sub_enable_th.daemon = True
        sub_enable_th.start()

    # ------------------- 发布逻辑 -------------------
    def publish_arm_state(self):
        arm_status = PiperStatusMsg()
        data = self.node.piper.GetArmStatus()
        arm_status.ctrl_mode = data.arm_status.ctrl_mode
        arm_status.arm_status = data.arm_status.arm_status
        arm_status.mode_feedback = data.arm_status.mode_feed
        arm_status.teach_status = data.arm_status.teach_status
        arm_status.motion_status = data.arm_status.motion_status
        arm_status.trajectory_num = data.arm_status.trajectory_num
        arm_status.err_code = data.arm_status.err_code

        err_status = data.arm_status.err_status
        arm_status.joint_1_angle_limit = err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def publish_arm_joint_and_gripper(self):
        # 机械臂关节角和夹爪位置
        # 原始数据是度*1000 -> 转弧度
        factor_deg2rad = 0.017444  # (3.14/180 ~ 0.017453，用作者代码就 0.017444)
        msgs = self.node.piper.GetArmJointMsgs()
        joint_0 = (msgs.joint_state.joint_1/1000)*factor_deg2rad
        joint_1 = (msgs.joint_state.joint_2/1000)*factor_deg2rad
        joint_2 = (msgs.joint_state.joint_3/1000)*factor_deg2rad
        joint_3 = (msgs.joint_state.joint_4/1000)*factor_deg2rad
        joint_4 = (msgs.joint_state.joint_5/1000)*factor_deg2rad
        joint_5 = (msgs.joint_state.joint_6/1000)*factor_deg2rad
        gripper_msg = self.node.piper.GetArmGripperMsgs()
        joint_6 = gripper_msg.gripper_state.grippers_angle/1000000

        speed_data = self.node.piper.GetArmHighSpdInfoMsgs()
        vel_0 = speed_data.motor_1.motor_speed/1000
        vel_1 = speed_data.motor_2.motor_speed/1000
        vel_2 = speed_data.motor_3.motor_speed/1000
        vel_3 = speed_data.motor_4.motor_speed/1000
        vel_4 = speed_data.motor_5.motor_speed/1000
        vel_5 = speed_data.motor_6.motor_speed/1000
        eff_0 = speed_data.motor_1.effort/1000
        eff_1 = speed_data.motor_2.effort/1000
        eff_2 = speed_data.motor_3.effort/1000
        eff_3 = speed_data.motor_4.effort/1000
        eff_4 = speed_data.motor_5.effort/1000
        eff_5 = speed_data.motor_6.effort/1000
        eff_6 = gripper_msg.gripper_state.grippers_effort/1000

        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.position = [joint_0,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6]
        self.joint_states.velocity = [vel_0,vel_1,vel_2,vel_3,vel_4,vel_5, 0.0]
        self.joint_states.effort = [eff_0,eff_1,eff_2,eff_3,eff_4,eff_5,eff_6]
        self.joint_pub.publish(self.joint_states)
        # self.arm_joint_status_rviz.publish(self.joint_states)

    def publish_arm_end_pose(self):
        end_data = self.node.piper.GetArmEndPoseMsgs().end_pose
        endpos = PoseStamped()
        endpos.pose.position.x = end_data.X_axis / 1e6
        endpos.pose.position.y = end_data.Y_axis / 1e6
        endpos.pose.position.z = end_data.Z_axis / 1e6
        roll_deg = end_data.RX_axis/1000
        pitch_deg = end_data.RY_axis/1000
        yaw_deg = end_data.RZ_axis/1000
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        quat = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        endpos.pose.orientation.x = quat[0]
        endpos.pose.orientation.y = quat[1]
        endpos.pose.orientation.z = quat[2]
        endpos.pose.orientation.w = quat[3]
        endpos.header.stamp = rospy.Time.now()
        self.end_pose_pub.publish(endpos)

        # 同时发布Euler
        euler_msg = PiperEulerPose()
        euler_msg.header.stamp = rospy.Time.now()
        euler_msg.x = end_data.X_axis / 1e6
        euler_msg.y = end_data.Y_axis / 1e6
        euler_msg.z = end_data.Z_axis / 1e6
        euler_msg.roll = roll_rad
        euler_msg.pitch = pitch_rad
        euler_msg.yaw = yaw_rad
        self.end_pose_euler_pub.publish(euler_msg)

    # ------------------- 订阅逻辑: 线程回调 -------------------
    def sub_pos_thread(self):
        rospy.Subscriber('pos_cmd', PosCmd, self.node.pos_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

    def sub_joint_thread(self):
        rospy.Subscriber('joint_ctrl_single', JointState, self.node.joint_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

    def sub_enable_thread(self):
        rospy.Subscriber('enable_flag', Bool, self.node.enable_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

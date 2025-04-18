#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
piper_ros_node.py: C_PiperRosNode类放在这里
"""

import time
import rospy
import threading
from piper_sdk import C_PiperInterface, C_PiperInterface_V2
import numpy as np

class C_PiperRosNode:
    """机械臂ros节点核心类"""
    def __init__(self, param_config, pub_sub_manager):
        """
        :param param_config: PiperParamConfig实例
        :param pub_sub_manager: PublishSubscribeManager实例
        """
        self.param_config = param_config
        self.factor = 1000 * 180 / np.pi
        self._enable_flag = False  # 是否使能
        self.pub_sub_manager = pub_sub_manager

        # 创建 Piper SDK 接口对象
        self.piper = C_PiperInterface(can_name=self.param_config.can_port)
        # 调用 Python 反射替换协议版本
        self.piper._C_PiperInterface__parser = C_PiperInterface_V2()
        self.piper.ConnectPort()
        # 默认初始动作
        self.piper.MotionCtrl_2(0x01, 0x01, 30, 0)

    def GetEnableFlag(self):
        """外部判断是否使能"""
        return self._enable_flag

    def Pubilsh(self):
        """
        主发布循环
        """
        rate = rospy.Rate(200)  # 200 Hz
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while not rospy.is_shutdown():
            if self.param_config.auto_enable:
                while not enable_flag:
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    # 查询电机状态
                    ls = self.piper.GetArmLowSpdInfoMsgs()
                    enable_flag = (ls.motor_1.foc_status.driver_enable_status and
                                   ls.motor_2.foc_status.driver_enable_status and
                                   ls.motor_3.foc_status.driver_enable_status and
                                   ls.motor_4.foc_status.driver_enable_status and
                                   ls.motor_5.foc_status.driver_enable_status and
                                   ls.motor_6.foc_status.driver_enable_status)
                    print("使能状态:", enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if enable_flag:
                        self._enable_flag = True
                    print("--------------------")
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
            if elapsed_time_flag:
                print("程序自动使能超时,退出程序")
                exit(0)

            # 发布各种消息
            self.pub_sub_manager.publish_arm_state()
            self.pub_sub_manager.publish_arm_end_pose()
            self.pub_sub_manager.publish_arm_joint_and_gripper()

            rate.sleep()

    # 以下几个回调函数直接在 piper_publish_subscribe.py 里使用
    def pos_callback(self, pos_data):
        if self.GetEnableFlag():
            self.piper.MotionCtrl_1(0x00,0x00,0x00)
            self.piper.MotionCtrl_2(0x01,0x00,50)
            factor = 180 / 3.1415926
            x = round(pos_data.x*1000)*1000
            y = round(pos_data.y*1000)*1000
            z = round(pos_data.z*1000)*1000
            rx = round(pos_data.roll*1000*factor)
            ry = round(pos_data.pitch*1000*factor)
            rz = round(pos_data.yaw*1000*factor)
            rospy.loginfo("Received PosCmd:")
            rospy.loginfo("x: %f", x)
            rospy.loginfo("y: %f", y)
            rospy.loginfo("z: %f", z)
            rospy.loginfo("roll: %f", rx)
            rospy.loginfo("pitch: %f", ry)
            rospy.loginfo("yaw: %f", rz)
            rospy.loginfo("gripper: %f", pos_data.gripper)
            rospy.loginfo("mode1: %d", pos_data.mode1)
            rospy.loginfo("mode2: %d", pos_data.mode2)
            self.piper.EndPoseCtrl(x,y,z,rx,ry,rz)
            gripper = round(pos_data.gripper*1e6)
            if pos_data.gripper>80000:
                gripper=80000
            if pos_data.gripper<0:
                gripper=0
            if self.param_config.gripper_exist:
                self.piper.GripperCtrl(abs(gripper),1000,0x01,0)
            self.piper.MotionCtrl_2(0x01,0x00,50)

    def joint_callback(self, joint_data):
        if not self.GetEnableFlag():
            return
        # 将关节 rad -> 0.001deg
        factor = self.factor
        arr_pos = list(joint_data.position)
        joint_0 = round(arr_pos[0]*factor)
        joint_1 = round(arr_pos[1]*factor)
        joint_2 = round(arr_pos[2]*factor)
        joint_3 = round(arr_pos[3]*factor)
        joint_4 = round(arr_pos[4]*factor)
        joint_5 = round(arr_pos[5]*factor)

        joint_6 = None
        if len(arr_pos)>=7:
            joint_6 = round(arr_pos[6]*1e6)
            joint_6 = int(joint_6*self.param_config.gripper_val_mutiple)
            if joint_6>80000:
                joint_6=80000
            if joint_6<0:
                joint_6=0

        # 如果用户速度非空
        if joint_data.velocity:
            all_zero = all(v==0 for v in joint_data.velocity)
        else:
            all_zero = True

        if not all_zero:
            if len(joint_data.velocity)==7:
                vel_all = round(joint_data.velocity[6])
                if vel_all>100: vel_all=100
                if vel_all<0: vel_all=0
                rospy.loginfo("vel_all: %d", vel_all)
                self.piper.MotionCtrl_2(0x01,0x01,vel_all)
            else:
                self.piper.MotionCtrl_2(0x01,0x01,50,0)
        else:
            self.piper.MotionCtrl_2(0x01,0x01,50,0)

        self.piper.JointCtrl(joint_0,joint_1,joint_2,joint_3,joint_4,joint_5)
        # 处理 gripper
        if self.param_config.gripper_exist and joint_6 is not None:
            if abs(joint_6)<200:
                joint_6=0
            # 力度
            if len(joint_data.effort)>=7:
                gripper_effort = joint_data.effort[6]
                gripper_effort = max(0.5, min(gripper_effort,3))
                gripper_effort = round(gripper_effort*1000)
                self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01,0)
            else:
                self.piper.GripperCtrl(abs(joint_6),1000,0x01,0)

    # TODO 待验证
    def gripper_callback(self, angle, effort):
        if effort:
            gripper_effort = effort
            gripper_effort = max(0.5, min(gripper_effort,3))
            gripper_effort = round(gripper_effort*1000)
            self.piper.GripperCtrl(angle, gripper_effort, 0x01,0)
        else:
            self.piper.GripperCtrl(angle,1000,0x01,0)

    # TODO 待验证
    def action_callback(self, jointIdx, pos, vel):
        if not self.GetEnableFlag():
            return
        # 将关节 rad -> 0.001deg
        factor = self.factor
        a_pos = pos * factor
        
        self.piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        # 电机序号，位置，速度，kd,kp,目标力矩
        self.piper.JointMitCtrl(jointIdx + 1, a_pos , vel, 10, 0.8, 0)


    def enable_callback(self, enable_flag):
        rospy.loginfo("Received enable flag:")
        rospy.loginfo("enable_flag: %s", enable_flag.data)
        if enable_flag.data:
            self._enable_flag = True
            self.piper.EnableArm(7)
            if self.param_config.gripper_exist:
                self.piper.GripperCtrl(0,1000,0x01,0)
        else:
            self._enable_flag = False
            self.piper.DisableArm(7)
            if self.param_config.gripper_exist:
                self.piper.GripperCtrl(0,1000,0x00,0)

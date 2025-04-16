#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

class PiperParamConfig:
    """
    用于获取并管理ROS参数的类
    """
    def __init__(self):
        # 默认值
        self.can_port = "can0"
        self.auto_enable = False
        self.gripper_exist = True
        self.rviz_ctrl_flag = False
        self.gripper_val_mutiple = 1.0

        # 加载参数
        self.load_params()

    def load_params(self):
        """
        从ROS param server获取参数
        """
        # can_port
        if rospy.has_param('~can_port'):
            self.can_port = rospy.get_param('~can_port')
            rospy.loginfo("%s is %s", rospy.resolve_name('~can_port'), self.can_port)
        else:
            rospy.loginfo("未找到can_port参数，使用默认值 %s", self.can_port)
            exit(0)
        
        # 是否自动使能，默认不自动使能
        if rospy.has_param('~auto_enable'):
            if rospy.get_param('~auto_enable'):
                self.auto_enable = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable'), self.auto_enable)

        # g是否有夹爪，默认为有
        if rospy.has_param('~gripper_exist'):
            if not rospy.get_param('~gripper_exist'):
                self.gripper_exist = False
        rospy.loginfo("%s is %s", rospy.resolve_name('~gripper_exist'), self.gripper_exist)

        # 是否是打开了rviz控制，默认为不是，如果打开了，gripper订阅的joint7关节消息会乘2倍-------已弃用
        if rospy.has_param('~rviz_ctrl_flag'):
            if rospy.get_param('~rviz_ctrl_flag'):
                self.rviz_ctrl_flag = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~rviz_ctrl_flag'), self.rviz_ctrl_flag)

        # 夹爪的数值倍数，默认为1
        if rospy.has_param('~gripper_val_mutiple'):
            val = rospy.get_param('~gripper_val_mutiple')
            if isinstance(val, (int, float)):
                if val <= 0:
                    rospy.logwarn("Invalid gripper_val_mutiple value: must be positive. Using default value of 1.")
                else:
                    self.gripper_val_mutiple = val
            else:
                rospy.logwarn("Invalid gripper_val_mutiple type. Expected int or float. Using 1.")
        else:
            rospy.logwarn("No gripper_val_mutiple param, using default value 1.")
        rospy.loginfo("%s is %s", rospy.resolve_name('~gripper_val_mutiple'), self.gripper_val_mutiple)


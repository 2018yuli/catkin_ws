#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from check_ros_master import check_ros_master
from piper_param_config import PiperParamConfig
from piper_publish_subscribe import PublishSubscribeManager
from piper_services import PiperServices
from piper_ros_node import C_PiperRosNode
from piper_action import PiperActions

if __name__ == '__main__':
    try:
        check_ros_master()  # 先检测ROS Master是否启动
        rospy.init_node('piper_ctrl_single_node', anonymous=True)

        # 加载参数
        param_config = PiperParamConfig()

        # 先初始化一个虚拟 node 实例(其中包含 piperSDK)
        piper_sdk = C_PiperRosNode(param_config, None)  # 先暂时传 None
        
        # 创建订阅发布管理，并传回node
        pub_sub_manager = PublishSubscribeManager(piper_sdk, param_config)
        
        # 让node记住pub_sub_manager
        piper_sdk.pub_sub_manager = pub_sub_manager

        # 创建Service
        piper_services = PiperServices(piper_sdk)

        # 开启 rviz 控制
        if param_config.rviz_ctrl_flag:
            piper_actions = PiperActions(piper_sdk)

        # 最后开始执行发布循环
        piper_sdk.Pubilsh()
    except rospy.ROSInterruptException:
        pass

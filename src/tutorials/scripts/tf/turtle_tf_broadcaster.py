#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入rospy库，这是ROS的Python客户端库
# 加载名为 'tutorials' 的包（这是一个示例包）
import roslib
roslib.load_manifest('tutorials')

import rospy

# 导入ROS的TF库，用于坐标系之间的转换广播
import tf
import turtlesim.msg

# 处理海龟位置消息的回调函数
def handle_turtle_pose(msg, turtlename):
    # 创建一个TransformBroadcaster对象，用于发布坐标变换
    br = tf.TransformBroadcaster()
    
    # 位置: (x, y, z)，z 固定为 0，因为海龟是在二维平面上
    translation = (msg.x, msg.y, 0)
    # 将海龟的朝向（theta）转化为四元数（quaternion）
    quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    # child frame in tf，海龟的名称（来自参数服务器）
    child = turtlename
    # parent frame in tf，这里是 "world"，表示海龟相对于世界坐标系的变换
    parent = "world"
    
    # 发布海龟的坐标变换信息
    br.sendTransform(translation, quaternion, rospy.Time.now(), child, parent)

if __name__ == '__main__':
    # 初始化ROS节点，节点名为 'turtle_tf_broadcaster'
    rospy.init_node('turtle_tf_broadcaster')
    
    # 获取参数服务器中的海龟名称（如果未设置参数，则使用默认值）
    # '~' 表示这是一个私有参数
    turtlename = rospy.get_param('~turtle', 'turtle1')
    
    # 订阅海龟位置的消息，消息类型是 `turtlesim.msg.Pose`
    rospy.Subscriber('/%s/pose' % turtlename,  # 订阅话题名称，话题名称是 `/turtle_name/pose`，turtle_name 是从参数中获取的
                     turtlesim.msg.Pose,  # 消息类型，表示海龟的位置和朝向
                     handle_turtle_pose,  # 回调函数，每次接收到消息时调用此函数
                     turtlename)  # 传递给回调函数的额外参数（海龟的名字）
    
    # 保持节点一直运行，直到ROS被关闭
    rospy.spin()  # 进入循环，等待回调函数处理消息

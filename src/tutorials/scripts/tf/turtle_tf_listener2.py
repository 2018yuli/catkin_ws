#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入rospy库，这是ROS的Python客户端库
# 加载名为 'tutorials' 的包（这是一个示例包）
import roslib
roslib.load_manifest('tutorials')

import rospy
import math
import tf
import geometry_msgs.msg
# 用于调用 turtlesim 仿真中的服务（比如生成新的海龟）
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    # 创建tf的监听器对象，用于获取坐标变换
    listener = tf.TransformListener()

    # 等待并确保 'spawn' 服务可用，'spawn' 服务用于在 turtlesim 仿真中生成新的海龟
    rospy.wait_for_service('spawn')
    # 创建一个服务代理，连接到 'spawn' 服务（生成海龟）
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # 调用 spawn 服务，在坐标 (4, 2) 处生成一个名为 'turtle2' 的新海龟
    spawner(4, 2, 0, 'turtle2')
    # 创建一个发布器，用于控制 'turtle2' 的速度，发布到 'turtle2/cmd_vel' 话题
    # 这个话题发布的是速度命令（Twist消息类型）
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    # 设置循环频率为10 Hz，即每秒发布10次命令
    rate = rospy.Rate(10.0)
    # 当ROS节点没有关闭时，循环执行
    while not rospy.is_shutdown():
        try:
            # listener 同时订阅 turtle2 和 carrot1 的 tf 坐标,然后计算得到 2 对于 1 的相对位置
            # 并发布 turtle2 对 carrot1 的运动向量
            # ---------------------------------------------------------------
            # 查找 '/turtle2' 和 '/carrot1' 坐标系之间的变换
            # listener.lookupTransform返回两个值：平移（trans）和旋转（rot）
            # 这里我们查询的是 turtel2 相对于 carrot1 的坐标变换
            (trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', rospy.Time(0))
        
        # 如果发生错误（例如：查找变换时的异常），则继续下一轮循环
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # 计算角速度：atan2 返回的是从 x 轴到 (x, y) 点的角度，范围从 -π 到 π
        # trans[0] 是 turtle2 到 carrot1 的 x 方向的位移
        # trans[1] 是 turtle2 到 carrot1 的 y 方向的位移
        # math.atan2(trans[1], trans[0]) 计算的是 turtle2 需要转动的角度
        #  - turtle2 当前位置到 carrot1 之间的直线的角度
        # 4 是一个系数，用来 放大 计算出来的角度。这意味着海龟调整方向的速度会更快
        angular = 4 * math.atan2(trans[1], trans[0])
        # 计算线速度：math.sqrt(trans[0] ** 2 + trans[1] ** 2) 计算的是 turtle2 到 carrot1 的 欧几里得距离
        # 0.5 * 距离,得到线速度
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # 创建一个Twist消息对象，用于存储线速度和角速度
        cmd = geometry_msgs.msg.Twist()
        # 设置线速度（沿 x 轴方向）
        cmd.linear.x = linear
        # 设置角速度（绕 z 轴旋转）
        cmd.angular.z = angular
        # 发布控制命令，控制 'turtle2' 运动
        turtle_vel.publish(cmd)

         # 等待下一个周期（即10Hz）
        rate.sleep()
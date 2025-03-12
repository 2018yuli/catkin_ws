#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

import rospy
from std_msgs.msg import String

def talker():
    
    # rospy.init_node 是 ROS 中初始化节点的函数。
    # 每个 ROS 程序必须有一个唯一的节点标识符，这个函数用于设置该节点的名称，
    # 并让节点与 ROS 主程序（Master）建立连接
    # anonymous=True：该参数表示每次运行节点时，ROS 会为其分配一个唯一的名称
    rospy.init_node('talker', anonymous=True)
    
    # 在 rospy.Publisher 中，queue_size 表示消息队列的大小，即发布者缓存的消息数量。
    # 如果队列已满，发布者将丢弃最旧的消息以腾出空间给新的消息
    #
    # queue_size=10 意味着如果消息发布速度过快，而订阅者没有及时处理消息，
    # 最多会缓存 10 条消息，超出的消息会被丢弃
    # 
    # ROS 1 并不会自动对消息做持久化，它的消息传输是临时的
    # 
    # queue_size 和超时无关。消息丢失是因为队列满了，而不是因为超时。
    # ROS 本身不会为发布的消息设置超时限制
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 10hz 表示每秒执行 10 次,（即休息 100 毫秒）
    rate = rospy.Rate(0.5) 
    # rospy.is_shutdown() 是 ROS 中检查节点是否已经被停止的一个函数。
    # 当 ROS Master 停止或者用户请求节点退出时，is_shutdown() 会返回 True
    #       while not rospy.is_shutdown() 是一个常见的 ROS 主循环结构
    #               ROS 的通信机制（例如话题、服务、参数等）底层可能是通过多线程或回调机制实现的
    #               因为这里进入了通信代码块,所以使用 while
    while not rospy.is_shutdown():
        # % 后面跟着的变量值会替换掉 %s
        hello_str = "hello world %s" % rospy.get_time()
        # ROS 的日志并不会落盘（除非你显式配置）。它将日志消息输出到终端（屏幕）以及 ROS 日志系统中
        # 日志文件，通常位于 ~/.ros/log/ 目录
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
        # C++ spinOnce()：类似于 rospy.spin()，它是一个非阻塞的函数，可以被频繁调用
        #   允许程序在循环中继续执行其他任务
        # rospy.spin()：它会阻塞当前线程，直到节点停止,这里并不能被调用
        # rospy.spin()
        rate.sleep()
        
if __name__  == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass
    finally:
        rospy.loginfo("bye")
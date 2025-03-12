#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

# from __future__ import 是一个特殊的导入语句，
# 允许你在当前版本的 Python 中使用未来版本的某些特性
# 作用是启用 Python 3 的 print() 函数
from __future__ import print_function

from tutorials.srv import AddTwoInts, AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
    rospy.sleep(8)
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server', log_level=rospy.DEBUG)
    # rospy.Service 本身并不会直接启动新的线程，但它会在 ROS 节点的事件循环中注册一个服务
    # 如果你在回调函数 handle_add_two_ints 中进行了耗时的操作或阻塞性操作，
    # 它会阻塞整个 ROS 事件循环的执行。
    #       为了避免这个问题，ROS 默认会为每个服务请求分配一个单独的线程来处理，
    #       保证服务的并发处理能力。
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    # rospy.spin() 是阻塞的，直到节点关闭
    rospy.spin()
    
if __name__ == "__main__":
    add_two_ints_server()
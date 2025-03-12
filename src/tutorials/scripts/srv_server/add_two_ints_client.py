#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

from __future__ import print_function

import sys
import rospy
from tutorials.srv import AddTwoInts

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints', timeout=2)
    try:
        # 当你创建 ServiceProxy 对象时，
        # 它与 ROS Master 建立联系，获取服务的信息（例如服务名称、请求和响应类型），
        # 并设置好通信管道
        add_two_ints = rospy.ServiceProxy('add_two_ints',  AddTwoInts)
        # 客户端会一直等待，直到响应返回或发生超时
        resp = add_two_ints.call(x, y)
        print("work done here")
        return resp.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)
        
def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s + %s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
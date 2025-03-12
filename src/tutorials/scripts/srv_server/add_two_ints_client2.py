#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 添加此行

from __future__ import print_function

import sys
import rospy
from tutorials.srv import AddTwoInts
import threading

def call_service(x, y, result_event, result, timeout_event):
    rospy.wait_for_service('add_two_ints', timeout=2)
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        if timeout_event.is_set():  # 如果超时，提前退出
            return  # 停止继续执行
        resp = add_two_ints(x, y)  # 调用服务
        result_event.set()  # 服务完成，设置事件标志
        result[0] = resp.sum  # 将结果存储到传递进来的 result 列表中
        print("I got the result:", resp.sum)
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)
        result_event.set()  # 服务调用失败，也设置事件标志

def add_two_ints_client(x, y):
    result_event = threading.Event()  # 用于表示服务调用是否完成
    timeout_event = threading.Event()  # 用于表示是否超时
    result = [None]  # 使用列表存储结果，以便修改
    service_thread = threading.Thread(target=call_service, args=(x, y, result_event, result, timeout_event))
    service_thread.start()
    service_thread.join(timeout=5)  # 等待线程，最多等待5秒

    if not result_event.is_set():  # 如果超时，线程没有完成
        print("Service call timed out.")
        timeout_event.set()  # 标记超时事件
        service_thread.join()  # 等待线程结束
    else:
        print("Result:", result[0])  # 如果结果已经返回，打印结果
    return result[0]  # 返回结果或 None
        
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
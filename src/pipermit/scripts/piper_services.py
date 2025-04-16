#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from std_srvs.srv import Trigger, TriggerResponse
from piper_msgs.srv import Enable, EnableResponse
from piper_msgs.srv import Gripper, GripperResponse
from piper_msgs.srv import GoZero, GoZeroResponse

class PiperServices:
    """
    用于管理并实现所有 Service 接口
    """
    def __init__(self, node):
        self.node = node
        # 创建 Service
        self.enable_service = rospy.Service('enable_srv', Enable, self.handle_enable_service)
        self.gripper_service = rospy.Service('gripper_srv', Gripper, self.handle_gripper_service)
        self.stop_service = rospy.Service('stop_srv', Trigger, self.handle_stop_service)
        self.reset_service = rospy.Service('reset_srv', Trigger, self.handle_reset_service)
        self.go_zero_service = rospy.Service('go_zero_srv', GoZero, self.handle_go_zero_service)

    # ---------- Service 回调函数 -----------
    def handle_enable_service(self, req):
        rospy.loginfo(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_list = []
            # 获取电机是否使能
            enable_list.append(self.node.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.node.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.node.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.node.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.node.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.node.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if(req.enable_request):
                enable_flag = all(enable_list)
                self.node.piper.EnableArm(7)
                self.node.piper.GripperCtrl(0,1000,0x01,0)
            else:
                enable_flag = any(enable_list)
                self.node.piper.DisableArm(7)
                self.node.piper.GripperCtrl(0,1000,0x02,0)
            print("使能状态:", enable_flag)
            self.node._enable_flag = enable_flag
            print("--------------------")
            if(enable_flag == req.enable_request):
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False
            if elapsed_time > timeout:
                print("超时....")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        response = enable_flag
        rospy.loginfo(f"Returning response: {response}")
        return EnableResponse(response)

    def handle_gripper_service(self, req):
        response = GripperResponse()
        response.code = 15999
        response.status = False
        if(self.node.param_config.gripper_exist):
            rospy.loginfo(f"-----------------------Gripper---------------------------")
            rospy.loginfo(f"Received request:")
            rospy.loginfo(f"PS: Piper should be enable.Please ensure piper is enable")
            rospy.loginfo(f"gripper_angle:{req.gripper_angle}, range is [0m, 0.07m]")
            rospy.loginfo(f"gripper_effort:{req.gripper_effort},range is [0.5N/m, 2N/m]")
            rospy.loginfo(f"gripper_code:{req.gripper_code}, range is [0, 1, 2, 3]")
            rospy.loginfo("0x00: Disable\n 0x01: Enable\n 0x03/0x02: Enable and clear error / Disable and clear error")
            rospy.loginfo(f"set_zero:{req.set_zero}, range is [0, 0xAE]")
            rospy.loginfo("0x00: Invalid value \n 0xAE: Set zero point")
            rospy.loginfo(f"-----------------------Gripper---------------------------")

            gripper_angle = round(max(0, min(req.gripper_angle, 0.07))*1e6)
            gripper_effort = round(max(0.5, min(req.gripper_effort,2))*1e3)
            if req.gripper_code not in [0x00,0x01,0x02,0x03]:
                rospy.logwarn("gripper_code should be in [0,1,2,3], default val=1")
                gripper_code = 1
                response.code = 15901
            else:
                gripper_code = req.gripper_code

            if req.set_zero not in [0x00,0xAE]:
                rospy.logwarn("set_zero should be in [0, 0xAE], default=0")
                set_zero = 0
                response.code = 15902
            else:
                set_zero = req.set_zero
            response.code = 15900
            self.node.piper.GripperCtrl(abs(gripper_angle), gripper_effort, gripper_code, set_zero)
            response.status = True
        else:
            rospy.logwarn("gripper_exist param is False.")
            response.code = 15903
            response.status = False
        rospy.loginfo(f"Returning GripperResponse: {response.code}, {response.status}")
        return response

    def handle_stop_service(self, req):
        response = TriggerResponse()
        response.success = False
        response.message = "stop piper failed"
        rospy.loginfo(f"-----------------------STOP---------------------------")
        rospy.loginfo(f"Stop piper.")
        rospy.loginfo(f"-----------------------STOP---------------------------")
        self.node.piper.MotionCtrl_1(0x01,0,0)
        response.success = True
        response.message = "stop piper success"
        rospy.loginfo(f"Returning StopResponse: {response.success}, {response.message}")
        return response

    def handle_reset_service(self, req):
        response = TriggerResponse()
        response.success = False
        response.message = "reset piper failed"
        rospy.loginfo(f"-----------------------RESET---------------------------")
        rospy.loginfo(f"reset piper.")
        rospy.loginfo(f"-----------------------RESET---------------------------")
        self.node.piper.MotionCtrl_1(0x02,0,0)
        response.success = True
        response.message = "reset piper success"
        rospy.loginfo(f"Returning resetResponse: {response.success}, {response.message}")
        return response

    def handle_go_zero_service(self, req):
        response = GoZeroResponse()
        response.status = False
        response.code = 151000
        rospy.loginfo(f"-----------------------GOZERO---------------------------")
        rospy.loginfo(f"piper go zero .")
        rospy.loginfo(f"-----------------------GOZERO---------------------------")
        if(req.is_mit_mode):
            self.node.piper.MotionCtrl_2(0x01, 0x01, 50, 0xAD)
        else:
            self.node.piper.MotionCtrl_2(0x01, 0x01, 50, 0)
        self.node.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        response.status = True
        response.code = 151001
        rospy.loginfo(f"Returning GoZeroResponse: {response.status}, {response.code}")
        return response

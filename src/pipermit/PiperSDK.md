## PiperSDK 接口

```
运动设置：

1) 主从模式
接口：MasterSlaveConfig
参数：linkage_config（模式配置）、feedback_offset（反馈偏移）、ctrl_offset（控制偏移）、linkage_offset（目标偏移）；无返回
示例：MasterSlaveConfig(0xFA, 0x10, 0x10, 0x10) # 设置主从模式

2) 使能电机
接口：EnableArm
参数：motor_num（电机号），enable_flag（使能标志）；无返回
示例：EnableArm(7, 0x02) # 使能所有电机

3) 电机状态查询
接口：GetArmHighSpdInfoMsgs
参数：无；返回高速电机状态
示例：GetArmHighSpdInfoMsgs() # 获取电机状态

4) 电机运动限制
接口：MotorAngleLimitMaxSpdSet
参数：motor_num（电机号），max_angle_limit（最大角度限制），min_angle_limit（最小角度限制），max_joint_spd（最大速度）；无返回
示例：MotorAngleLimitMaxSpdSet(1,0x7FFF,0x7FFF,3000) # 设置1号电机限制

5) 关节设置0点
接口：JointConfig
参数：joint_num（关节号），set_zero（是否设零点），acc_param_is_effective（加速度是否生效），max_joint_acc（最大加速度），clear_err（清错）；无返回
示例：JointConfig(1,0xAE,0,500,0) # 将关节1当前角度设为0

6) 查询末端速度
接口：GetCurrentEndVelAndAccParam
参数：无；返回末端速度加速度
示例：GetCurrentEndVelAndAccParam() # 获取末端速度参数

7) 末端运动限制
接口：EndSpdAndAccParamSet
参数：end_max_linear_vel（最大线速度），end_max_angular_vel（最大角速度），end_max_linear_acc（最大线加速度），end_max_angular_acc（最大角加速度）；无返回
示例：EndSpdAndAccParamSet(1000,200,1000,200) # 设置末端运动限制

8) 末端防护等级
接口：CrashProtectionConfig
参数：joint_1_protection_level ~ joint_6_protection_level（各关节碰撞等级0~8）；无返回
示例：CrashProtectionConfig(0,0,0,0,0,0) # 关闭碰撞检测


运动控制：

1) 控制模式
接口：ModeCtrl
参数：ctrl_mode（控制模式），move_mode（运动模式），move_spd_rate_ctrl（速度百分比），is_mit_mode（是否mit）；无返回
示例：ModeCtrl(0x01,0x01,50,0x00) # 切换关节控制模式

2) 运动模式
接口：MotionCtrl_2
参数：ctrl_mode（控制模式），move_mode（运动模式），move_spd_rate_ctrl（速度百分比），is_mit_mode（是否mit），residence_time（停留时间），installation_pos（安装位）；无返回
示例：MotionCtrl_2(0x01,0x02,50,0x00,0,0x00) # 切换直线运动

3) 笛卡尔运动
接口：EndPoseCtrl
参数：X、Y、Z、RX、RY、RZ（单位0.001）；无返回
示例：EndPoseCtrl(1000,0,300,0,0,0) # 末端位姿移动

4) 关节角度运动
接口：JointCtrl
参数：joint_1 ~ joint_6（单位0.001度）；无返回
示例：JointCtrl(0,10000,0,0,0,0) # 移动关节2到10°

5) 圆弧运动
接口：MoveCAxisUpdateCtrl
参数：instruction_num（0x00~0x03）；无返回
示例：MoveCAxisUpdateCtrl(0x01) # 设置圆弧起点

6) 夹爪控制
接口：GripperCtrl
参数：gripper_angle（角度），gripper_effort（力矩），gripper_code（使能），set_zero（零点）；无返回
示例：GripperCtrl(1000,500,0x01,0x00) # 控制夹爪

7) mit运动
接口：JointMitCtrl
参数：motor_num，pos_ref，vel_ref，kp，kd，t_ref；无返回
示例：JointMitCtrl(1,0.5,0,10,0.8,0) # 关节1MIT控制


反馈：

1) 电机反馈
接口：GetArmStatus
参数：无；返回机械臂当前状态
示例：GetArmStatus() # 查看机械臂状态

2) 关节角度反馈
接口：GetArmJointMsgs
参数：无；返回各关节角度
示例：GetArmJointMsgs() # 查看关节角度

3) 夹爪反馈
接口：GetArmGripperMsgs
参数：无；返回夹爪角度与扭矩
示例：GetArmGripperMsgs() # 查看夹爪状态
```
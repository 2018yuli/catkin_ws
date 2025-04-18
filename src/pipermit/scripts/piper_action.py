#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from .interpolator import TrajectoryInterpolator

class PiperActions:
    """
    用于管理并实现所有 Action 接口
    """
    def __init__(self, node):
        self.node = node
        # 创建 Action
        self._action_server = actionlib.SimpleActionServer('/arm_controllers/follow_joint_trajectory',
                                                           FollowJointTrajectoryAction,
                                                           execute_cb=self.execute_cb,
                                                           auto_start=False)
       
        self.joint_names = None
        self.current_positions = None
        self.current_velocities = None
        self._action_server.start()
        rospy.loginfo("JointTrajectoryActionServer started and ready to receive goals.")
    
    # ---------- Action 回调函数 -----------
    def execute_cb(self, goal):
        """Callback to accept and execute a new trajectory goal."""
        feedback = FollowJointTrajectoryFeedback()
        result = FollowJointTrajectoryResult()

        # 验证目标
        traj = goal.trajectory
        if len(traj.points) == 0:
            rospy.logerr("Received empty trajectory goal.")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._action_server.set_aborted(result, "Empty trajectory")
            return
        
        # first goal，设置关节名称和当前状态（如果未初始化）
        if self.joint_names is None:
            self.joint_names = list(traj.joint_names)
            # Initialize current position to first point if provided, otherwise zeros
            if len(traj.points[0].positions) == len(self.joint_names):
                # If first trajectory point time_from_start is 0, assume robot is already at this start position
                self.current_positions = list(traj.points[0].positions) if traj.points[0].time_from_start.to_sec() == 0.0 else [0.0] * len(self.joint_names)
            else:
                # Mismatch in joint count
                rospy.logerr("Joint count mismatch between goal and server.")
                result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
                self._action_server.set_aborted(result, "Joint name mismatch")
                return
            # Assume starting from rest (zero velocity)
            self.current_velocities = [0.0] * len(self.joint_names)
        else:
            # If subsequent goal, ensure joint names match
            if list(traj.joint_names) != self.joint_names:
                rospy.logerr("Joint names in the trajectory do not match the expected joint names.")
                result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
                self._action_server.set_aborted(result, "Joint name mismatch")
                return

        # If the first point in the trajectory is not at time 0, insert a starting point at current state (time 0)
        points = list(traj.points)
        if points[0].time_from_start.to_sec() > 1e-6:
            # Create a new initial point at t=0 with current positions
            init_point = JointTrajectoryPoint()
            init_point.positions = list(self.current_positions)
            init_point.velocities = [0.0] * len(self.joint_names)
            init_point.accelerations = [0.0] * len(self.joint_names)
            init_point.time_from_start = rospy.Duration(0.0)
            points.insert(0, init_point)
            rospy.loginfo("Inserted initial point at current state to start trajectory.")
       
        interpolator = TrajectoryInterpolator(self.joint_names, points)
        total_time = points[-1].time_from_start.to_sec()
        rospy.loginfo("Executing trajectory for %.3f seconds..." % total_time)
        
         # Simulation loop: step through trajectory time and update joint states
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)  # 100 Hz control loop
        last_feedback_time = rospy.Time.now()
        success = True

        # Prepare feedback message static fields
        feedback.joint_names = self.joint_names
        t = 0.0
        while t < total_time:
            # Check for preemption or cancellation
            if self._action_server.is_preempt_requested() or rospy.is_shutdown():
                success = False
                break

            # Compute desired position and velocity at current time t
            desired_positions, desired_velocities = interpolator.get_state(t)
            # Compute control command for each joint using PID (error = desired_pos - current_pos)
            for j in range(len(self.joint_names)):
                error = desired_positions[j] - self.current_positions[j]
                # Commanded joint velocity = feedforward (desired velocity) + P-correction
                joint_cmd_vel = desired_velocities[j]
                # Update simulated joint state: integrate velocity to get new position
                self.current_positions[j] += joint_cmd_vel * (1.0/100.0)
                self.current_velocities[j] = joint_cmd_vel
                # (Hardware interface point: send joint_cmd_vel to the actual motor controller here)
                self.node.mit_callback(j, desired_positions[j], joint_cmd_vel)
            
            # Publish feedback at a reduced rate (e.g., 20 Hz)
            current_time = rospy.Time.now()
            if (current_time - last_feedback_time) >= rospy.Duration(0.05):
                # Populate feedback message with desired, actual, and error
                feedback.header.stamp = current_time
                # Desired state at time t
                feedback.desired.positions = desired_positions
                feedback.desired.velocities = desired_velocities
                feedback.desired.time_from_start = rospy.Duration(t)
                # Actual state (current simulated positions/velocities)
                feedback.actual.positions = list(self.current_positions)
                feedback.actual.velocities = list(self.current_velocities)
                feedback.actual.time_from_start = rospy.Duration(t)
                # Position error for feedback (desired - actual)
                error_positions = [dp - cp for dp, cp in zip(desired_positions, self.current_positions)]
                feedback.error.positions = error_positions
                feedback.error.time_from_start = rospy.Duration(t)
                self._action_server.publish_feedback(feedback)
                last_feedback_time = current_time
            
             # Wait for next control step
            rate.sleep()
            # Update relative time `t` based on real elapsed time to maintain sync with wall-clock
            t = (rospy.Time.now() - start_time).to_sec()
        
        # If preempted or aborted
        if not success:
            if rospy.is_shutdown():
                return  # Node shutting down, exit immediately
            rospy.loginfo("Trajectory execution preempted or cancelled.")
            # Set the action state to preempted
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._action_server.set_preempted(result, "Trajectory preempted")
            # Save current state for next goal start
            # (Robot would be at self.current_positions now)
            return

        # Ensure final position is exactly reached (correct any residual error)
        final_desired_pos, _ = interpolator.get_state(total_time)
        self.current_positions = list(final_desired_pos)
        self.current_velocities = [0.0] * len(self.joint_names)

        rospy.loginfo("Trajectory execution completed successfully.")
        # Populate result (success)
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self._action_server.set_succeeded(result, "Trajectory executed successfully")


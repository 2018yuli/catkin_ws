#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

class PIDController:
    """Simple PID controller for one joint."""
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._prev_error = 0.0
        self._integral = 0.0

    def reset(self):
        """Reset the integrator and previous error."""
        self._prev_error = 0.0
        self._integral = 0.0

    def compute(self, error, dt):
        """
        Compute the control output for a given position error and timestep.
        This output can be interpreted as a velocity command to move the joint.
        """
        # Integral term
        self._integral += error * dt
        # Derivative term (difference in error)
        derivative = 0.0
        if dt > 0:
            derivative = (error - self._prev_error) / dt
        # PID output
        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
        # Save error for next derivative calculation
        self._prev_error = error
        return output

class TrajectoryInterpolator:
    """
    Interpolates a joint trajectory (with multiple points) for all joints.
    Chooses linear, cubic, or quintic interpolation for each segment based on provided velocities/accelerations.
    """
    def __init__(self, joint_names, trajectory_points):
        self.joint_names = list(joint_names)
        self.n_joints = len(joint_names)
        # Extract trajectory points data (positions, velocities, accelerations, times)
        # Ensure points are sorted by time_from_start
        self.points = []
        for pt in trajectory_points:
            # time_from_start is a rospy.Duration; convert to float seconds
            t = pt.time_from_start.to_sec()
            pos = list(pt.positions)
            vel = list(pt.velocities) if pt.velocities else []  # empty list if not provided
            acc = list(pt.accelerations) if pt.accelerations else []
            self.points.append({'t': t, 'pos': pos, 'vel': vel, 'acc': acc})
        # Sort points by time (they usually are already sorted)
        self.points.sort(key=lambda p: p['t'])
        # Build interpolation segments
        self.segments = []
        for i in range(len(self.points) - 1):
            p0 = self.points[i]
            p1 = self.points[i+1]
            t0, t1 = p0['t'], p1['t']
            T = t1 - t0
            if T <= 0:
                rospy.logwarn("Trajectory points are not strictly increasing in time.")
                T = max(T, 1e-6)  # prevent division by zero
            # Decide interpolation type based on data availability
            has_vel0 = (len(p0['vel']) == self.n_joints)
            has_vel1 = (len(p1['vel']) == self.n_joints)
            has_acc0 = (len(p0['acc']) == self.n_joints)
            has_acc1 = (len(p1['acc']) == self.n_joints)
            # Determine interpolation: quintic if both vel & acc provided, cubic if any vel provided, else linear
            if has_acc0 and has_acc1:
                itype = 'quintic'
            elif has_vel0 or has_vel1:
                itype = 'cubic'
            else:
                itype = 'linear'
            # Prepare segment data
            seg = {'t0': t0, 't1': t1, 'type': itype, 'coeffs': [None]*self.n_joints}
            if itype == 'linear':
                # Precompute slope for each joint
                seg['pos0'] = p0['pos']  # starting positions
                seg['slopes'] = [(p1['pos'][j] - p0['pos'][j]) / T for j in range(self.n_joints)]
            elif itype == 'cubic':
                for j in range(self.n_joints):
                    q0 = p0['pos'][j]
                    q1 = p1['pos'][j]
                    # If velocity not provided at an endpoint, assume 0.0 (rest) for interpolation
                    v0 = p0['vel'][j] if has_vel0 else 0.0
                    v1 = p1['vel'][j] if has_vel1 else 0.0
                    # Solve for cubic polynomial coefficients: pos(t) = D + C*t + B*t^2 + A*t^3
                    D = q0
                    C = v0
                    D1 = q1 - q0 - v0 * T    # helper: position difference minus initial vel contribution
                    D2 = v1 - v0             # helper: total change in velocity over segment
                    # Coefficients for cubic (A*t^3 + B*t^2 + C*t + D)
                    A = (D2 * T - 2.0 * D1) / (T**3)
                    B = (3.0 * D1 - D2 * T) / (T**2)
                    seg['coeffs'][j] = (D, C, B, A)
            else:  # quintic
                for j in range(self.n_joints):
                    q0 = p0['pos'][j]
                    q1 = p1['pos'][j]
                    v0 = p0['vel'][j] if has_vel0 else 0.0
                    v1 = p1['vel'][j] if has_vel1 else 0.0
                    a0 = p0['acc'][j] if has_acc0 else 0.0
                    a1 = p1['acc'][j] if has_acc1 else 0.0
                    # Quintic polynomial coefficients: pos(t) = A0 + A1*t + A2*t^2 + A3*t^3 + A4*t^4 + A5*t^5
                    A0 = q0
                    A1 = v0
                    A2 = 0.5 * a0
                    # Solve for A3, A4, A5 using boundary conditions at t=0 and t=T
                    # Boundary conditions:
                    # pos(T)=q1, vel(T)=v1, acc(T)=a1 (with pos(0)=q0, vel(0)=v0, acc(0)=a0)
                    # Derived formula coefficients for quintic segment:
                    T2 = T * T
                    T3 = T2 * T
                    T4 = T3 * T
                    T5 = T4 * T
                    # Solve for the remaining coefficients:
                    A3 = (  2.0*q0 - 2.0*q1 + v1*T + 3.0*a0*T2/2.0 - a1*T2/2.0 + 4.0*v0*T ) * (2.0 / T3)
                    A4 = ( -3.0*q0 + 3.0*q1 - 2.0*v1*T - 3.0*a0*T2 + a1*T2 - 4.0*v0*T ) * (2.0 / T4)
                    A5 = (  6.0*q0 - 6.0*q1 + 3.0*v1*T +      a0*T2 - a1*T2 + 6.0*v0*T ) * (1.0 / T5)
                    seg['coeffs'][j] = (A0, A1, A2, A3, A4, A5)
            self.segments.append(seg)

    def get_state(self, time_from_start):
        """
        Compute the desired joint positions and velocities at the given time_from_start (in seconds).
        Returns a tuple (positions_list, velocities_list).
        """
        t = time_from_start
        # Clamp time to bounds
        if t <= self.segments[0]['t0']:
            # Before or at start: hold initial position, zero velocity
            return (list(self.segments[0]['pos0'] if 'pos0' in self.segments[0] else self.points[0]['pos']),
                    [0.0] * self.n_joints)
        if t >= self.segments[-1]['t1']:
            # After or at end: hold final position, zero velocity
            return (list(self.points[-1]['pos']), [0.0] * self.n_joints)
        # Find the current segment for time t
        seg = None
        for s in self.segments:
            if s['t0'] <= t < s['t1']:
                seg = s
                break
        if seg is None:
            seg = self.segments[-1]
        # Time into the current segment
        dt = t - seg['t0']
        # Compute positions and velocities for each joint based on segment type
        positions = [0.0] * self.n_joints
        velocities = [0.0] * self.n_joints
        if seg['type'] == 'linear':
            # Linear interpolation: constant velocity
            for j in range(self.n_joints):
                positions[j] = seg['pos0'][j] + seg['slopes'][j] * dt
                velocities[j] = seg['slopes'][j]
        elif seg['type'] == 'cubic':
            for j in range(self.n_joints):
                D, C, B, A = seg['coeffs'][j]
                positions[j] = D + C*dt + B*(dt**2) + A*(dt**3)
                velocities[j] = C + 2*B*dt + 3*A*(dt**2)
        else:  # quintic
            for j in range(self.n_joints):
                A0, A1, A2, A3, A4, A5 = seg['coeffs'][j]
                positions[j] = A0 + A1*dt + A2*(dt**2) + A3*(dt**3) + A4*(dt**4) + A5*(dt**5)
                velocities[j] = A1 + 2*A2*dt + 3*A3*(dt**2) + 4*A4*(dt**3) + 5*A5*(dt**4)
        return (positions, velocities)

class JointTrajectoryActionServer:
    def __init__(self):
        rospy.init_node('joint_trajectory_action_server')
        # Create the SimpleActionServer for FollowJointTrajectoryAction
        self._action_server = actionlib.SimpleActionServer('follow_joint_trajectory',
                                                           FollowJointTrajectoryAction,
                                                           execute_cb=self.execute_cb,
                                                           auto_start=False)
        # Initialize storage for current joint state (for simulation)
        self.joint_names = None
        self.current_positions = None
        self.current_velocities = None
        self._action_server.start()
        rospy.loginfo("JointTrajectoryActionServer started and ready to receive goals.")

    def execute_cb(self, goal):
        """Callback to accept and execute a new trajectory goal."""
        # Initialize result and feedback messages
        feedback = FollowJointTrajectoryFeedback()
        result = FollowJointTrajectoryResult()

        # Validate the goal
        traj = goal.trajectory
        if len(traj.points) == 0:
            rospy.logerr("Received empty trajectory goal.")
            result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._action_server.set_aborted(result, "Empty trajectory")
            return

        # On first goal, set joint names and current state if not initialized
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

        # Create a TrajectoryInterpolator for the trajectory points
        interpolator = TrajectoryInterpolator(self.joint_names, points)
        total_time = points[-1].time_from_start.to_sec()

        # Setup PID controllers for each joint (using small gains for simulation)
        controllers = [PIDController(Kp=5.0, Ki=0.0, Kd=0.0) for _ in range(len(self.joint_names))]
        # Reset integrators (especially important if this is a new goal after a previous one)
        for pid in controllers:
            pid.reset()

        # Prepare feedback message static fields
        feedback.joint_names = self.joint_names

        # Simulation loop: step through trajectory time and update joint states
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)  # 100 Hz control loop
        last_feedback_time = rospy.Time.now()
        success = True

        rospy.loginfo("Executing trajectory for %.3f seconds..." % total_time)
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
                # PID output as velocity correction
                vel_correction = controllers[j].compute(error, 1.0/100.0)  # using dt = 0.01 (100Hz)
                # Commanded joint velocity = feedforward (desired velocity) + P-correction
                joint_cmd_vel = desired_velocities[j] + vel_correction
                # Update simulated joint state: integrate velocity to get new position
                self.current_positions[j] += joint_cmd_vel * (1.0/100.0)
                self.current_velocities[j] = joint_cmd_vel
                # (Hardware interface point: send joint_cmd_vel to the actual motor controller here)

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

if __name__ == '__main__':
    server = JointTrajectoryActionServer()
    rospy.spin()

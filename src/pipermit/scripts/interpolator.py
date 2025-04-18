#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

class TrajectoryInterpolator:
    def __init__(self, joint_names, trajectory_points):
        self.joint_names = list(joint_names)
        self.n_joints = len(joint_names)
        # Extract trajectory points data (positions, velocities, times)
        self.points = []
        for pt in trajectory_points:
            t = pt.time_from_start.to_sec()  # Convert to float seconds
            pos = list(pt.positions)
            vel = list(pt.velocities) if pt.velocities else []  # Empty list if not provided
            self.points.append({'t': t, 'pos': pos, 'vel': vel})
        # Sort points by time
        self.points.sort(key=lambda p: p['t'])
        # Build interpolation segments (only linear interpolation)
        self.segments = []
        for i in range(len(self.points) - 1):
            p0 = self.points[i]
            p1 = self.points[i+1]
            t0, t1 = p0['t'], p1['t']
            T = t1 - t0
            if T <= 0:
                rospy.logwarn("Trajectory points are not strictly increasing in time.")
                T = max(T, 1e-6)  # Prevent division by zero
            # Precompute slope for position and velocity for each joint
            seg = {'t0': t0, 't1': t1, 'pos0': p0['pos'], 'vel0': p0['vel']}
            seg['slopes_pos'] = [(p1['pos'][j] - p0['pos'][j]) / T for j in range(self.n_joints)]
            seg['slopes_vel'] = [(p1['vel'][j] - p0['vel'][j]) / T for j in range(self.n_joints)] if p0['vel'] and p1['vel'] else [0.0] * self.n_joints
            self.segments.append(seg)

    def get_state(self, time_from_start):
        """
        Compute the desired joint positions and velocities at the given time_from_start (in seconds).
        Returns a tuple (positions_list, velocities_list).
        """
        t = time_from_start
        # Clamp time to bounds
        if t <= self.segments[0]['t0']:
            # Before or at start: hold initial position and velocity
            return (list(self.segments[0]['pos0']),
                    list(self.segments[0]['vel0']))
        if t >= self.segments[-1]['t1']:
            # After or at end: hold final position and velocity
            return (list(self.points[-1]['pos']),
                    [0.0] * self.n_joints)
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
        # Compute positions and velocities for each joint based on linear interpolation
        positions = [0.0] * self.n_joints
        velocities = [0.0] * self.n_joints
        for j in range(self.n_joints):
            # Linear interpolation for position
            positions[j] = seg['pos0'][j] + seg['slopes_pos'][j] * dt
            # Linear interpolation for velocity
            velocities[j] = seg['vel0'][j] + seg['slopes_vel'][j] * dt
        return (positions, velocities)

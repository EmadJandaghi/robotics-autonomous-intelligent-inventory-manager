#!/usr/bin/env python3
import time
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectorySender:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(
            node,
            FollowJointTrajectory,
            '/deepmind_bot_head_controller/follow_joint_trajectory'
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('[TrajectorySender] Head controller not available.')
            raise RuntimeError('Action server not available')

    def send_trajectory(self, joint_names, positions, duration_sec):
        """Fire-and-forget: send trajectory goal asynchronously without spinning."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        goal.trajectory.points.append(point)

        # Send goal asynchronously
        future = self.client.send_goal_async(goal)

        def _goal_done(fut):
            try:
                handle = fut.result()
                if handle.accepted:
                    self.node.get_logger().info('[TrajectorySender] Goal accepted.')
                else:
                    self.node.get_logger().warn('[TrajectorySender] Goal rejected.')
            except Exception as e:
                self.node.get_logger().error(f'[TrajectorySender] Goal send error: {e}')

        future.add_done_callback(_goal_done)

        # Give controller a moment to start (non-blocking)
        time.sleep(1.0)
        return True

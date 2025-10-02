#!/usr/bin/env python3
"""ROS 2 trajectory tool implemented in Python."""

from __future__ import annotations

import collections
from typing import Deque, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf_transformations


class TrajectoryToolNode(Node):
    """Aggregate odometry streams into path messages for visualization."""

    def __init__(self) -> None:
        super().__init__('trajectory_tool')

        self.target_topic = self.declare_parameter('target_topic', 'odometry/filtered').value
        self.reference_topic = self.declare_parameter('reference_topic', 'ground_truth').value
        self.target_path_topic = self.declare_parameter('target_path_topic', 'target_path').value
        self.reference_path_topic = self.declare_parameter('reference_path_topic', 'reference_path').value
        self.difference_path_topic = self.declare_parameter('difference_path_topic', 'trajectory_difference').value
        self.publish_reference_path = bool(self.declare_parameter('publish_reference_path', True).value)
        self.publish_difference_path = bool(self.declare_parameter('publish_difference_path', True).value)
        self.max_path_length = int(self.declare_parameter('max_path_length', 0).value)
        self.sync_tolerance = float(self.declare_parameter('sync_tolerance', 0.05).value)
        self.reference_buffer_duration = float(
            self.declare_parameter('reference_buffer_duration', 5.0).value
        )

        if self.max_path_length < 0:
            self.get_logger().warn('max_path_length must be non-negative. Resetting to unlimited.')
            self.max_path_length = 0

        if self.sync_tolerance <= 0.0:
            self.get_logger().warn('sync_tolerance must be positive. Resetting to 0.05 s.')
            self.sync_tolerance = 0.05

        if 0.0 < self.reference_buffer_duration < self.sync_tolerance:
            self.get_logger().warn(
                'reference_buffer_duration should be greater than sync_tolerance. '
                'Increasing to match the tolerance.'
            )
            self.reference_buffer_duration = self.sync_tolerance

        self.target_path = Path()
        self.reference_path = Path()
        self.difference_path = Path()

        self.reference_buffer: Deque[Odometry] = collections.deque()

        self.target_path_pub = self.create_publisher(Path, self.target_path_topic, 10)
        self.reference_path_pub = None
        self.difference_path_pub = None

        if self.reference_topic and self.publish_reference_path:
            self.reference_path_pub = self.create_publisher(Path, self.reference_path_topic, 10)

        if self.reference_topic and self.publish_difference_path:
            self.difference_path_pub = self.create_publisher(Path, self.difference_path_topic, 10)

        self.target_sub = self.create_subscription(
            Odometry, self.target_topic, self.target_callback, 10
        )

        self.reference_sub = None
        if self.reference_topic:
            self.reference_sub = self.create_subscription(
                Odometry, self.reference_topic, self.reference_callback, 10
            )

    def target_callback(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._append_pose(self.target_path, pose)
        self._prune_path(self.target_path)

        if self.target_path_pub.get_subscription_count() > 0:
            self.target_path_pub.publish(self.target_path)

        if not self.reference_topic:
            return

        target_time = Time.from_msg(msg.header.stamp)
        self._prune_reference_buffer(target_time)
        reference = self._find_best_reference(target_time)
        if reference is None:
            return

        if self.publish_difference_path and self.difference_path_pub:
            diff_pose = self._compute_difference(msg, reference)
            if diff_pose is not None:
                self._append_pose(self.difference_path, diff_pose)
                self._prune_path(self.difference_path)
                if self.difference_path_pub.get_subscription_count() > 0:
                    self.difference_path_pub.publish(self.difference_path)

    def reference_callback(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._append_pose(self.reference_path, pose)
        self._prune_path(self.reference_path)

        if self.reference_path_pub and self.reference_path_pub.get_subscription_count() > 0:
            self.reference_path_pub.publish(self.reference_path)

        self.reference_buffer.append(msg)
        self._prune_reference_buffer(Time.from_msg(msg.header.stamp))

    def _compute_difference(self, target: Odometry, reference: Odometry) -> Optional[PoseStamped]:
        try:
            target_matrix = self._pose_to_matrix(target.pose.pose)
            reference_matrix = self._pose_to_matrix(reference.pose.pose)
        except ValueError as exc:  # pragma: no cover - defensive programming
            self.get_logger().warn(f'Unable to compute trajectory difference: {exc}')
            return None

        diff_matrix = tf_transformations.concatenate_matrices(
            tf_transformations.inverse_matrix(reference_matrix), target_matrix
        )
        translation = tf_transformations.translation_from_matrix(diff_matrix)
        quaternion = tf_transformations.quaternion_from_matrix(diff_matrix)

        pose = PoseStamped()
        pose.header.stamp = target.header.stamp
        pose.header.frame_id = reference.header.frame_id
        pose.pose.position.x = float(translation[0])
        pose.pose.position.y = float(translation[1])
        pose.pose.position.z = float(translation[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        return pose

    def _pose_to_matrix(self, pose) -> np.ndarray:
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        matrix = tf_transformations.quaternion_matrix(quaternion)
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z
        return matrix

    def _find_best_reference(self, target_time: Time) -> Optional[Odometry]:
        if not self.reference_buffer:
            return None

        best_match = None
        best_delta = self.sync_tolerance
        for candidate in self.reference_buffer:
            delta = abs((target_time - Time.from_msg(candidate.header.stamp)).nanoseconds) / 1e9
            if delta <= best_delta:
                best_delta = delta
                best_match = candidate
        return best_match

    def _prune_reference_buffer(self, stamp: Time) -> None:
        if self.reference_buffer_duration <= 0.0:
            return

        minimum_time = stamp - Duration(seconds=self.reference_buffer_duration)
        while self.reference_buffer:
            oldest = self.reference_buffer[0]
            if Time.from_msg(oldest.header.stamp) < minimum_time:
                self.reference_buffer.popleft()
            else:
                break

    def _append_pose(self, path: Path, pose: PoseStamped) -> None:
        path.header.frame_id = pose.header.frame_id
        path.header.stamp = pose.header.stamp
        path.poses.append(pose)

    def _prune_path(self, path: Path) -> None:
        if self.max_path_length <= 0:
            return

        overflow = len(path.poses) - self.max_path_length
        if overflow > 0:
            del path.poses[:overflow]


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryToolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

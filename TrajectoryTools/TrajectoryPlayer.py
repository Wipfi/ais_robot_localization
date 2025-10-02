#!/usr/bin/env python3
"""Replay recorded trajectory samples as odometry messages."""

from __future__ import annotations

import pathlib
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import yaml


@dataclass
class TrajectorySample:
    """A single trajectory sample loaded from disk."""

    time_from_start: float
    frame_id: str
    child_frame_id: str
    pose: Pose
    twist: Twist


def _load_pose(data: dict) -> Pose:
    pose = Pose()
    position = data.get('position', {})
    pose.position.x = float(position.get('x', 0.0))
    pose.position.y = float(position.get('y', 0.0))
    pose.position.z = float(position.get('z', 0.0))

    orientation = data.get('orientation', {})
    pose.orientation.x = float(orientation.get('x', 0.0))
    pose.orientation.y = float(orientation.get('y', 0.0))
    pose.orientation.z = float(orientation.get('z', 0.0))
    pose.orientation.w = float(orientation.get('w', 1.0))
    return pose


def _load_twist(data: dict) -> Twist:
    twist = Twist()
    linear = data.get('linear', {})
    twist.linear.x = float(linear.get('x', 0.0))
    twist.linear.y = float(linear.get('y', 0.0))
    twist.linear.z = float(linear.get('z', 0.0))

    angular = data.get('angular', {})
    twist.angular.x = float(angular.get('x', 0.0))
    twist.angular.y = float(angular.get('y', 0.0))
    twist.angular.z = float(angular.get('z', 0.0))
    return twist


class TrajectoryPlayer(Node):
    """Publish odometry messages for a recorded trajectory."""

    def __init__(self) -> None:
        super().__init__('trajectory_player')

        self.trajectory_file = self.declare_parameter('trajectory_file', '').value
        self.output_topic = self.declare_parameter('output_topic', 'trajectory_player/odometry').value
        self.publish_rate = float(self.declare_parameter('publish_rate', 30.0).value)
        self.loop = bool(self.declare_parameter('loop', False).value)
        self.default_frame_id = self.declare_parameter('frame_id', '').value
        self.default_child_frame_id = self.declare_parameter('child_frame_id', '').value

        self.samples = self._load_samples(self.trajectory_file)
        if not self.samples:
            raise RuntimeError('No trajectory samples available for playback')

        self.publisher = self.create_publisher(Odometry, str(self.output_topic), 10)

        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self._on_timer)

        self._start_time: Optional[Time] = None
        self._index = 0

    def _load_samples(self, file_path: str) -> List[TrajectorySample]:
        if not file_path:
            raise RuntimeError('trajectory_file parameter must be provided for TrajectoryPlayer')

        path = pathlib.Path(str(file_path)).expanduser()
        if not path.exists():
            raise RuntimeError(f'Trajectory file {path} does not exist')

        with path.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}

        if isinstance(data, dict) and 'trajectory' in data:
            entries = data['trajectory']
        elif isinstance(data, list):
            entries = data
        else:
            raise RuntimeError('Trajectory file must contain a list of samples or a "trajectory" key')

        samples: List[TrajectorySample] = []
        for entry in entries:
            if not isinstance(entry, dict):
                continue

            time_from_start = float(entry.get('time_from_start', entry.get('stamp', 0.0)))
            frame_id = str(entry.get('frame_id', self.default_frame_id or 'map'))
            child_frame_id = str(entry.get('child_frame_id', self.default_child_frame_id or 'base_link'))
            pose = _load_pose(entry.get('pose', {}))
            twist = _load_twist(entry.get('twist', {}))
            samples.append(TrajectorySample(time_from_start, frame_id, child_frame_id, pose, twist))

        samples.sort(key=lambda sample: sample.time_from_start)
        return samples

    def _on_timer(self) -> None:
        if self._start_time is None:
            self._start_time = self.get_clock().now()
            self._index = 0

        now = self.get_clock().now()

        while self._index < len(self.samples):
            sample = self.samples[self._index]
            target_time = self._start_time + Duration(seconds=float(sample.time_from_start))
            if now < target_time:
                break

            msg = self._build_odometry(sample, now)
            self.publisher.publish(msg)
            self._index += 1

        if self._index >= len(self.samples) and self.loop:
            self._start_time = now
            self._index = 0

    def _build_odometry(self, sample: TrajectorySample, stamp: Time) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = sample.frame_id
        msg.child_frame_id = sample.child_frame_id

        msg.pose = PoseWithCovariance()
        msg.pose.pose = sample.pose
        msg.twist = TwistWithCovariance()
        msg.twist.twist = sample.twist
        return msg


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    try:
        node = TrajectoryPlayer()
    except RuntimeError as exc:
        rclpy.logging.get_logger('trajectory_player').error(str(exc))
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

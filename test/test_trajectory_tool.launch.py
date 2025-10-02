#!/usr/bin/env python3
"""Integration test for the trajectory tool node."""

import math
import time
import unittest
from collections import deque

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node

import rclpy
from nav_msgs.msg import Odometry, Path


def generate_test_description():
    trajectory_node = Node(
        package='robot_localization',
        executable='trajectory_tool.py',
        name='trajectory_tool',
        parameters=[{
            'target_topic': 'target_odometry',
            'reference_topic': 'reference_odometry',
            'target_path_topic': 'target_path',
            'reference_path_topic': 'reference_path',
            'difference_path_topic': 'difference_path',
            'sync_tolerance': 0.25,
            'reference_buffer_duration': 2.0,
            'max_path_length': 10,
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        trajectory_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'trajectory_node': trajectory_node}


class TestTrajectoryTool(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('trajectory_tool_tester')
        self.target_pub = self.node.create_publisher(Odometry, 'target_odometry', 10)
        self.reference_pub = self.node.create_publisher(Odometry, 'reference_odometry', 10)
        self.target_paths = deque()
        self.reference_paths = deque()
        self.difference_paths = deque()
        self.target_sub = self.node.create_subscription(Path, 'target_path', self._target_callback, 10)
        self.reference_sub = self.node.create_subscription(Path, 'reference_path', self._reference_callback, 10)
        self.difference_sub = self.node.create_subscription(Path, 'difference_path', self._difference_callback, 10)
        self._wait_for_connections()

    def tearDown(self):
        self.node.destroy_subscription(self.target_sub)
        self.node.destroy_subscription(self.reference_sub)
        self.node.destroy_subscription(self.difference_sub)
        self.node.destroy_publisher(self.target_pub)
        self.node.destroy_publisher(self.reference_pub)
        self.node.destroy_node()

    def _target_callback(self, msg: Path) -> None:
        self.target_paths.append(msg)

    def _reference_callback(self, msg: Path) -> None:
        self.reference_paths.append(msg)

    def _difference_callback(self, msg: Path) -> None:
        self.difference_paths.append(msg)

    def _wait_for_connections(self, timeout=5.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            pubs_ready = self.target_pub.get_subscription_count() > 0 and \
                self.reference_pub.get_subscription_count() > 0
            subs_ready = self.target_sub.get_publisher_count() > 0 and \
                self.reference_sub.get_publisher_count() > 0 and \
                self.difference_sub.get_publisher_count() > 0
            if pubs_ready and subs_ready:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('Timed out waiting for trajectory tool connections')

    def _create_odometry(self, x=0.0, y=0.0, yaw=0.0) -> Odometry:
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        if yaw != 0.0:
            half_yaw = yaw * 0.5
            odom.pose.pose.orientation.z = math.sin(half_yaw)
            odom.pose.pose.orientation.w = math.cos(half_yaw)
        else:
            odom.pose.pose.orientation.w = 1.0
        return odom

    def _publish_pair(self, target: Odometry, reference: Odometry) -> None:
        timestamp = self.node.get_clock().now().to_msg()
        target.header.stamp = timestamp
        reference.header.stamp = timestamp
        self.target_pub.publish(target)
        self.reference_pub.publish(reference)

    def _wait_for_path(self, queue: deque, timeout=5.0) -> Path:
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if queue:
                return queue.pop()
        self.fail('Did not receive expected path message')

    def test_trajectory_tool_publishes_paths(self):
        """Trajectory tool should publish target, reference, and difference paths."""
        target = self._create_odometry(x=2.0, y=-1.0)
        target.header.frame_id = 'odom'
        target.child_frame_id = 'base_link'

        reference = self._create_odometry(x=1.0, y=-2.0)
        reference.header.frame_id = 'map'
        reference.child_frame_id = 'reference_base'

        self._publish_pair(target, reference)

        target_path = self._wait_for_path(self.target_paths)
        reference_path = self._wait_for_path(self.reference_paths)
        difference_path = self._wait_for_path(self.difference_paths)

        self.assertEqual(len(target_path.poses), 1)
        self.assertEqual(target_path.header.frame_id, 'odom')
        self.assertAlmostEqual(target_path.poses[0].pose.position.x, 2.0, places=6)
        self.assertAlmostEqual(target_path.poses[0].pose.position.y, -1.0, places=6)

        self.assertEqual(len(reference_path.poses), 1)
        self.assertEqual(reference_path.header.frame_id, 'map')
        self.assertAlmostEqual(reference_path.poses[0].pose.position.x, 1.0, places=6)
        self.assertAlmostEqual(reference_path.poses[0].pose.position.y, -2.0, places=6)

        self.assertEqual(len(difference_path.poses), 1)
        self.assertEqual(difference_path.header.frame_id, 'map')
        diff_pose = difference_path.poses[0].pose
        self.assertAlmostEqual(diff_pose.position.x, 1.0, places=6)
        self.assertAlmostEqual(diff_pose.position.y, 1.0, places=6)
        self.assertAlmostEqual(diff_pose.orientation.w, 1.0, places=6)
        self.assertAlmostEqual(diff_pose.orientation.z, 0.0, places=6)


if __name__ == '__main__':
    unittest.main()

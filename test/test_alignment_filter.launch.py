#!/usr/bin/env python3
"""Integration test for the alignment filter node."""

import math
import time
import unittest
from collections import deque

import launch
import launch_testing
import launch_testing.actions
import launch_testing.tools
from launch_ros.actions import Node

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def generate_test_description():
    alignment = Node(
        package='robot_localization',
        executable='alignment_filter_node',
        name='alignment_filter',
        parameters=[{
            'reference_topic': 'reference_odometry',
            'subject_topic': 'subject_odometry',
            'aligned_topic': 'aligned_odometry',
            'transform_topic': 'alignment_transform',
            'fixed_frame_id': 'map',
            'moving_frame_id': 'odom',
            'publish_tf': False,
            'publish_transform_topic': True,
            'publish_aligned_odometry': True,
            'timeout': 0.0,
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        alignment,
        launch_testing.actions.ReadyToTest(),
    ]), {'alignment_action': alignment}


class TestAlignmentFilter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('alignment_filter_tester')
        self.transforms = deque()
        self.aligned_messages = deque()
        self.transform_sub = self.node.create_subscription(
            TransformStamped,
            'alignment_transform',
            self._transform_callback,
            10,
        )
        self.aligned_sub = self.node.create_subscription(
            Odometry,
            'aligned_odometry',
            self._aligned_callback,
            10,
        )
        self.reference_pub = self.node.create_publisher(Odometry, 'reference_odometry', 10)
        self.subject_pub = self.node.create_publisher(Odometry, 'subject_odometry', 10)
        self._wait_for_connections()

    def tearDown(self):
        self.node.destroy_subscription(self.transform_sub)
        self.node.destroy_subscription(self.aligned_sub)
        self.node.destroy_publisher(self.reference_pub)
        self.node.destroy_publisher(self.subject_pub)
        self.node.destroy_node()

    def _wait_for_connections(self, timeout=5.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            publishers_ready = self.reference_pub.get_subscription_count() > 0 and \
                self.subject_pub.get_subscription_count() > 0
            subscribers_ready = self.transform_sub.get_publisher_count() > 0 and \
                self.aligned_sub.get_publisher_count() > 0
            if publishers_ready and subscribers_ready:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('Timed out waiting for alignment filter connections')

    def _transform_callback(self, msg: TransformStamped) -> None:
        self.transforms.append(msg)

    def _aligned_callback(self, msg: Odometry) -> None:
        self.aligned_messages.append(msg)

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

    def _publish_pair(self, subject: Odometry, reference: Odometry) -> None:
        timestamp = self.node.get_clock().now().to_msg()
        subject.header.stamp = timestamp
        reference.header.stamp = timestamp
        self.subject_pub.publish(subject)
        self.reference_pub.publish(reference)

    def _wait_for_transform(self, timeout=5.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.transforms:
                return self.transforms.popleft()
        self.fail('Did not receive alignment transform')

    def _wait_for_aligned(self, timeout=5.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.aligned_messages:
                return self.aligned_messages.popleft()
        self.fail('Did not receive aligned odometry')

    def test_alignment_filter_computes_transform(self):
        """Alignment filter should publish the transform and aligned odometry."""
        subject = self._create_odometry(x=1.0, y=0.0)
        subject.header.frame_id = 'odom'
        subject.child_frame_id = 'base_link'
        reference = self._create_odometry(x=4.0, y=0.0)
        reference.header.frame_id = 'map'
        reference.child_frame_id = 'base_link_map'

        self._publish_pair(subject, reference)
        transform = self._wait_for_transform()
        self.assertEqual(transform.header.frame_id, 'map')
        self.assertEqual(transform.child_frame_id, 'odom')
        self.assertAlmostEqual(transform.transform.translation.x, 3.0, places=6)
        self.assertAlmostEqual(transform.transform.translation.y, 0.0, places=6)
        self.assertAlmostEqual(transform.transform.translation.z, 0.0, places=6)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0, places=6)

        aligned = self._wait_for_aligned()
        self.assertEqual(aligned.header.frame_id, 'map')
        self.assertAlmostEqual(aligned.pose.pose.position.x, reference.pose.pose.position.x, places=6)
        self.assertAlmostEqual(aligned.pose.pose.position.y, reference.pose.pose.position.y, places=6)
        self.assertAlmostEqual(aligned.pose.pose.position.z, reference.pose.pose.position.z, places=6)


@launch_testing.post_shutdown_test()
class TestAlignmentFilterShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        proc_info.assertWaitForShutdown(process_matcher=launch_testing.tools.proc_any())

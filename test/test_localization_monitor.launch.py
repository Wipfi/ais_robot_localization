#!/usr/bin/env python3
"""Integration test for the localization monitor node."""

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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def generate_test_description():
    monitor = Node(
        package='robot_localization',
        executable='localization_monitor_node',
        name='localization_monitor',
        parameters=[{
            'target_topic': 'test_target',
            'reference_topic': 'test_reference',
            'monitor_frequency': 20.0,
            'position_tolerance': 0.2,
            'orientation_tolerance': 0.2,
            'linear_velocity_tolerance': 0.5,
            'angular_velocity_tolerance': 0.5,
            'timeout': 0.0,
            'publish_diagnostics': True,
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        monitor,
        launch_testing.actions.ReadyToTest(),
    ]), {'monitor_action': monitor}


class TestLocalizationMonitor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('localization_monitor_tester')
        self.diagnostics = deque()
        self.diag_sub = self.node.create_subscription(
            DiagnosticArray,
            'diagnostics',
            self._diagnostic_callback,
            10,
        )
        self.target_pub = self.node.create_publisher(Odometry, 'test_target', 10)
        self.reference_pub = self.node.create_publisher(Odometry, 'test_reference', 10)
        self._wait_for_subscribers()

    def tearDown(self):
        self.node.destroy_subscription(self.diag_sub)
        self.node.destroy_publisher(self.target_pub)
        self.node.destroy_publisher(self.reference_pub)
        self.node.destroy_node()

    def _wait_for_subscribers(self, timeout=5.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            if (self.target_pub.get_subscription_count() > 0 and
                    self.reference_pub.get_subscription_count() > 0):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('Timed out waiting for localization monitor subscriptions')

    def _diagnostic_callback(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.diagnostics.append(msg.status[0])

    def _create_odometry(self, x=0.0, y=0.0, yaw=0.0) -> Odometry:
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation = Quaternion(w=1.0)
        if yaw != 0.0:
            half_yaw = yaw * 0.5
            odom.pose.pose.orientation.z = math.sin(half_yaw)
            odom.pose.pose.orientation.w = math.cos(half_yaw)
        return odom

    def _publish_pair(self, target: Odometry, reference: Odometry) -> None:
        now = self.node.get_clock().now().to_msg()
        target.header.stamp = now
        reference.header.stamp = now
        self.target_pub.publish(target)
        self.reference_pub.publish(reference)

    def _wait_for_status(self, predicate, timeout=5.0):
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            while self.diagnostics:
                status = self.diagnostics.popleft()
                if predicate(status):
                    return status
        self.fail('Did not receive expected diagnostic status')

    def test_localization_monitor_reports_warnings(self):
        """Monitor should report OK for matching states and WARN for divergent ones."""
        ok_target = self._create_odometry(x=1.0, y=2.0)
        ok_reference = self._create_odometry(x=1.0, y=2.0)
        self._publish_pair(ok_target, ok_reference)
        status = self._wait_for_status(lambda s: any(v.key == 'position_error' for v in s.values))
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, 'Localization is within tolerance')

        self.diagnostics.clear()
        warn_target = self._create_odometry(x=3.0, y=0.0)
        warn_reference = self._create_odometry(x=0.0, y=0.0)
        self._publish_pair(warn_target, warn_reference)

        def warn_predicate(status_msg):
            for value in status_msg.values:
                if value.key == 'position_error' and float(value.value) > 2.5:
                    return True
            return False

        warn_status = self._wait_for_status(warn_predicate)
        self.assertEqual(warn_status.level, DiagnosticStatus.WARN)
        self.assertIn('Position error exceeds tolerance', warn_status.message)


@launch_testing.post_shutdown_test()
class TestLocalizationMonitorShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        proc_info.assertWaitForShutdown(process_matcher=launch_testing.tools.proc_any())

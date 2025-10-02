robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

## Additional Nodes

### Localization monitor

The `localization_monitor` node subscribes to a target localization topic and a
reference topic and checks the positional, angular, and velocity deviations
between the two. Thresholds are configurable via ROS parameters and the node can
emit diagnostics to integrate with existing monitoring pipelines.

Launch example: `ros2 launch robot_localization localization_monitor.launch.py`

An integration test covering the diagnostic behavior is available via
`colcon test --packages-select robot_localization --event-handlers console_direct+`.

### Alignment filter

The `alignment_filter` node compares two odometry streams and computes the
transform aligning the moving frame with the reference frame. The node can
publish the resulting transform to TF, republish the aligned odometry in the
reference frame, and expose the alignment transform as a standalone topic.

Launch example: `ros2 launch robot_localization alignment_filter.launch.py`

The `test_alignment_filter.launch.py` integration test verifies the transform
and aligned odometry outputs when the filter receives simple synthetic data.

### Trajectory tool

The `trajectory_tool` node aggregates odometry streams into `nav_msgs/Path`
messages for visualization and comparison. It always publishes a path for the
target localization topic and can optionally track a reference stream and the
relative pose between the target and reference states when both are available.

Launch example: `ros2 launch robot_localization trajectory_tool.launch.py`

The `TrajectoryTools/TrajectoryPlayer.py` utility can replay recorded YAML
trajectory samples as odometry messages for the node to visualize. Provide the
file path with the `trajectory_file` parameter when starting the player with
`ros2 run robot_localization TrajectoryPlayer.py` or via a launch file.

The `test_trajectory_tool.launch.py` integration test exercises the path
outputs and verifies the difference path matches the expected offset between
the target and reference odometry.

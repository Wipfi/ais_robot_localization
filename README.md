ais_robot_localization
======================

ais_robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

## AIS extensions for ROS 2

This fork adds ROS 2 ports of AIS-specific tools that were previously only available on the
`noetic-devel` branch:

* **Localization monitor**: evaluates relative pose error (RPE) between a local odometry source
  and a global reference, publishes diagnostic statistics, and streams matching path markers.
* **Alignment filter**: continuously aligns the local odometry frame to the global frame by running
  a Kabsch-based optimization over a sliding window of localization monitor samples and optionally
  publishes the transform as TF.
* **NavSat preprocessing node**: fuses GNSS odometry with external heading estimates before feeding
  them into the filters and optionally emits an IMU message with the corrected orientation.
* **Launch and visualization assets**: a turnkey launch file to start the full monitoring pipeline
  in ROS 2 and RViz layouts to inspect trajectories, diagnostics, and alignment results.

### Launch and scripting support

### Localization monitor overview

The localization monitor subscribes to a locally integrated odometry stream (for example,
`/odometry/local`) and a globally referenced solution (GNSS, motion capture, etc.). It synchronizes
their pose histories, performs a point-to-point alignment, and periodically computes relative pose
error statistics. The node publishes the aligned trajectories on the `Global_Path` and
`Local_Path` topics so they can be inspected in RViz alongside the diagnostic messages.

The `RPE_Values` topic carries a `std_msgs/msg/Float32MultiArray` with four entries that summarize
the latest comparison window:

1. Path-length-weighted translational error in meters. The monitor integrates the translational
   residual along the synchronized trajectories (trapezoidal rule) over a sliding subtrajectory
   window that ends at the newest synchronized sample. Dividing the integral by the traveled
   distance in that window keeps the statistic independent of the chosen reference length while
   still letting longer, better-populated segments influence the result more than short, noisy
   samples.
2. Path-length-weighted rotational error in radians, computed with the same integration scheme and
   normalization over the same newest subtrajectory window. The value therefore remains independent
   of the reference length yet emphasizes sustained orientation drift near the current time horizon
   instead of treating each pose as an equal-weighted sample.
3. Maximum translational error in meters, evaluated over the current comparison window by
   measuring every pose in that window against the latest synchronized pose.
4. Maximum rotational error in radians computed with the same newest-pose reference so the
   bound reflects the worst current deviation instead of historical outliers.

Values are set to `inf` when insufficient data is available (e.g., during startup) so downstream
consumers can detect gaps.

- `localization_monitor.launch.py` starts the full AIS monitoring pipeline and exposes launch
  arguments to toggle the navsat preprocessing, alignment filter, navsat transform stages, or RViz
  as well as to remap the involved topics.
- `localization_monitor_node.launch.py`, `alignment_filter.launch.py`, and
  `navsat_preprocessing.launch.py` provide component-level launch files for targeted debugging or
  integration into larger applications.
- Parameter presets for each component live in `params/localization_monitor.yaml`,
  `params/alignment_filter.yaml`, and `params/navsat_preprocessing.yaml` and can be overridden with
  custom YAML files.
- `scripts/localization_monitor_pipeline.sh` mirrors the ROS 1 byobu helpers by creating a tmux
  session that launches the pipeline (with optional arguments) and opens RViz with the
  `config/MonitorAnalysis.rviz` layout when available. Pass `start_rviz:=false` if you want to skip
  the visualization window. A focused configuration for alignment debugging lives in
  `config/AlignmentFilter.rviz`.

#### Example: `localization_monitor_node.launch.py`

To launch only the localization monitor component without any preprocessing stages, invoke the
dedicated launch file and supply the odometry topics expected by your system:

```bash
ros2 launch ais_robot_localization localization_monitor_node.launch.py \
  localization_monitor.global_odom_topic:=/my/global/odometry \
  localization_monitor.local_odom_topic:=/my/local/odometry \
  localization_monitor.rpe_topic:=/localization_monitor/RPE_Values
```

Each argument remaps the parameters on the `localization_monitor_node` so you can point the
diagnostics at custom odometry sources while keeping the rest of the pipeline disabled.

### Alignment filter 2D projection mode

The alignment filter exposes a `2D_mode` parameter that projects the estimated global transform,
aligned odometry, and transformed local trajectory onto the XY plane. When enabled, all published
poses use zero altitude and yaw-only orientation (roll and pitch are forced to zero). The parameter
defaults to `false` to preserve the original 3D behavior.

To enable the mode when using the bundled pipeline or launch files, override the alignment filter
parameters, for example:

```bash
ros2 launch ais_robot_localization localization_monitor.launch.py use_alignment_filter:=true \
  alignment_filter_params:=/path/to/custom_alignment_filter.yaml
```

With a custom parameter file containing:

```yaml
alignment_filter:
  ros__parameters:
    2D_mode: true
```

Alternatively, you can set the parameter directly when launching the standalone alignment filter:

```bash
ros2 launch ais_robot_localization alignment_filter.launch.py alignment_filter.2D_mode:=true
```

Both approaches will make the alignment filter publish planar transforms while the rest of the
pipeline remains unchanged.


- `py_tools/TrajectoryTools` contains the AIS trajectory generator, player, and helper notebooks.
  The `TrajectoryPlayer.py` script now uses ROS 2 (`rclpy`) publishers and TF broadcasters so you can
  replay dill-based paths directly into Humble environments while visualizing them with the bundled
  Matplotlib UI.



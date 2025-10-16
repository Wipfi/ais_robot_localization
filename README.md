ais_robot_localization
======================

ais_robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/robot_localization

## AIS extensions for ROS 2

This fork adds ROS 2 tools based on a Master Thesis (https://doi.org/10.3217/2eb04-1vk32):

* **Localization monitor**: evaluates relative pose error (RPE) between a local odometry source
  and a global reference, publishes diagnostic statistics, and streams matching path markers.
* **Alignment filter**: continuously aligns the local odometry frame to the global frame by running
  a Kabsch-based optimization over a sliding window of localization monitor samples and optionally
  publishes the transform as TF.
* **NavSat preprocessing node**: Fuses GNSS odometry with orientation extimate. This is robot specific for the robot mercator of the TU-Graz

### Launch Files
There are currently still some robot specific launch files

### Localization monitor overview
The localization monitor node implements the consistency evaluation of two odometry data as described in in the paper "A Trajectory Consistency Metric for GNSS Anomaly Detection with LiDAR Odometry" ( https://doi.org/10.34749/3061-0710.2025.25).
The localization monitor subscribes to a locally integrated odometry stream (for example,
`/odometry/local`) and a globally referenced solution (GNSS, motion capture, etc.). It synchronizes
their pose histories, performs a point-to-point alignment, and on movment or periodically (parameter: `time_based` defaults to ``false``) computes relative pose
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

Values are set to `inf` when insufficient data is available (e.g., during startup).

### Parameters
| **Parameter**         | **Type** | **Default** | **Description**                                                                                                                      |
| --------------------- | -------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `dist_cum_threshold`  | `double` | `15.0`      | Maximum cumulative trajectory length (in meters) considered for analysis. Older poses are discarded once this threshold is exceeded. |
| `time_based`          | `bool`   | `false`     | If `true`, pose cleanup is based on timestamp differences instead of cumulative distance.                                            |
| `publish_rate`        | `double` | `0.5`       | Used if timebased is set to true: Frequency (in Hz) at which monitoring results are published.                                                                         |
| `max_poses_threshold` | `int`    | `500`       | Maximum number of stored poses before old entries are removed. This prevents memory overflow in long sessions.                       |
| `global_frame`        | `string` | `map`       | The global reference frame used for absolute localization (e.g., GNSS or map frame).                                                 |
| `local_frame`         | `string` | `odom`      | The local reference frame used for relative motion estimation (e.g., LiDAR odometry).                                                |


#### Example: `localization_monitor_node.launch.py`

To launch only the localization monitor component without any preprocessing stages, invoke the
dedicated launch file and supply the odometry topics expected by your system:

```bash
ros2 launch ais_robot_localization localization_monitor_node.launch.py \
  global_odom_topic:=/my/global/odometry \
  local_odom_topic:=/my/local/odometry \
  rpe_topic:=/localization_monitor/RPE_Values
```

Each argument remaps the parameters on the `localization_monitor_node` so you can point the
diagnostics at custom odometry sources while keeping the rest of the pipeline disabled.

#### Alignment Filter
ToDo
robot_localization
==================

robot_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

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

- `localization_monitor.launch.py` starts the full AIS monitoring pipeline and exposes launch
  arguments to toggle the navsat preprocessing, alignment filter, and navsat transform stages or to
  remap the involved topics.
- `localization_monitor_node.launch.py`, `alignment_filter.launch.py`, and
  `navsat_preprocessing.launch.py` provide component-level launch files for targeted debugging or
  integration into larger applications.
- Parameter presets for each component live in `params/localization_monitor.yaml`,
  `params/alignment_filter.yaml`, and `params/navsat_preprocessing.yaml` and can be overridden with
  custom YAML files.
- `scripts/localization_monitor_pipeline.sh` mirrors the ROS 1 byobu helpers by creating a tmux
  session that launches the pipeline (with optional arguments) and opens RViz with the
  `config/MonitorAnalysis.rviz` layout when available. A focused configuration for alignment
  debugging lives in `config/AlignmentFilter.rviz`.


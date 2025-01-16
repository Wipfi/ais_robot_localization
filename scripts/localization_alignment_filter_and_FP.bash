#!/bin/bash

# Start Byobu session
byobu new-session -d -s Localization_Session -n DLO

# Window 1: DLO
byobu rename-window -t Localization_Session:0 "DLO"
byobu send-keys -t Localization_Session:0 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization dlo.launch pointcloud_topic:=/livox/lidar imu_topic:=/imu/data' C-m


# Window 2: Data Conversion
byobu new-window -t Localization_Session -n "Data Conversion"
byobu send-keys -t Localization_Session:1 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization navsat_preprocessing_geo_kombi.launch' C-m

# Window 3: Filtering
byobu new-window -t Localization_Session -n "Fusion"
byobu send-keys -t Localization_Session:2 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization alignment_filter_geo_kombi_standard.launch' C-m

byobu split-window -v -t Localization_Session:2
byobu send-keys -t Localization_Session:2.1 'source ~/tedusar_ws/devel/setup.bash;
roslaunch ais_robot_localization localization_monitor.launch' C-m

# Window 4: Fixposition
byobu new-window -t Localization_Session -n "Fixposition"
byobu send-keys -t Localization_Session:3 'source ~/fixposition_ws/devel/setup.bash; 
roslaunch fixposition_driver_ros1 tcp.launch' C-m

byobu split-window -v -t Localization_Session:3
byobu send-keys -t Localization_Session:3.1 'source ~/tedusar_ws/devel/setup.bash;
roscd ais_robot_localization; cd scripts; python3 fixposition_frame_connector.py' C-m

byobu split-window -v -t Localization_Session:3
byobu send-keys -t Localization_Session:3.2 'source ~/tedusar_ws/devel/setup.bash;
roscd ais_robot_localization; cd scripts; python3 llh_to_utm_converter.py' C-m

# Attach to the Byobu session
byobu attach -t Localization_Session
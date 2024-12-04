#!/bin/bash

# Start Byobu session
byobu new-session -d -s Localization_Session -n DLO

# Window 1: DLO
byobu rename-window -t Localization_Session:0 "DLO"
byobu send-keys -t Localization_Session:0 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization dlo.launch pointcloud_topic:=/livox/lidar imu_topic:=/imu/data'  C-m


# Window 2: Data Conversion
byobu new-window -t Localization_Session -n "Data Conversion"
byobu send-keys -t Localization_Session:1 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization navsat_preprocessing_geo_kombi.launch' C-m

# Window 2: Filtering
byobu new-window -t Localization_Session -n "Fusion"
byobu send-keys -t Localization_Session:2 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization ekf_dlo_geo_kombi_standard.launch' C-m

# Window 4: Visualization
byobu new-window -t Localization_Session -n "Visualization"
byobu send-keys -t Localization_Session:3 'source ~/tedusar_ws/devel/setup.bash; 
roscd ais_robot_localization; cd config;
rviz -d satelite_ekf.rviz' C-m

# Attach to the Byobu session
byobu attach -t Localization_Session
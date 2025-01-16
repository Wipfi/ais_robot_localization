#!/bin/bash

# Start Byobu session
byobu new-session -d -s Localization_Session -n DLO

# Window 2: Data Conversion
byobu new-window -t Localization_Session -n "Data Conversion"
byobu send-keys -t Localization_Session:0 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization navsat_preprocessing_sim.launch' C-m

# Window 3: Filtering
byobu new-window -t Localization_Session -n "Fusion"
byobu send-keys -t Localization_Session:1 'source ~/tedusar_ws/devel/setup.bash; 
roslaunch ais_robot_localization alignment_filter_sim.launch' C-m

byobu split-window -v -t Localization_Session:1
byobu send-keys -t Localization_Session:1.1 'source ~/tedusar_ws/devel/setup.bash;
roslaunch ais_robot_localization localization_monitor.launch' C-m

# Window 4: Visualization
byobu new-window -t Localization_Session -n "Visualization"
byobu send-keys -t Localization_Session:2 'source ~/tedusar_ws/devel/setup.bash; 
roscd ais_robot_localization; cd config;
rviz' C-m

# Attach to the Byobu session
byobu attach -t Localization_Session
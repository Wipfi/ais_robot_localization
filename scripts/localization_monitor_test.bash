#!/bin/bash

# Start Byobu session
byobu new-session -d -s Localization_Session
byobu send-keys -t Localization_Session "tmux set-option -g mouse on" C-m

# Window 2: Data Conversion
#byobu new-window -t Localization_Session -n "Data Conversion"
byobu send-keys -t Localization_Session:0 'source /NOETIC/ais_loc_ws/devel/setup.bash; 
roslaunch ais_robot_localization navsat_preprocessing_sim.launch' C-m

# Window 2: Filtering
byobu new-window -t Localization_Session -n "Fusion"
byobu send-keys -t Localization_Session:1 'source /NOETIC/ais_loc_ws/devel/setup.bash; 
roslaunch ais_robot_localization ekf_dlo_sim.launch' C-m

# Window 4: Visualization
byobu new-window -t Localization_Session -n "Visualization"
byobu send-keys -t Localization_Session:2 'source /NOETIC/ais_loc_ws/devel/setup.bash; 
rviz -d $(rospack find ais_robot_localization)/config/MonitorAnalysis.rviz' C-m

# Window 2: Monitor
byobu new-window -t Localization_Session -n "Monitores"
byobu send-keys -t Localization_Session:3 'source /NOETIC/ais_loc_ws/devel/setup.bash;
roslaunch ais_robot_localization localization_monitor_py_test.launch' C-m


byobu split-window -v -t Localization_Session:3
byobu send-keys -t Localization_Session:3.1 'source /NOETIC/ais_loc_ws/devel/setup.bash;
roslaunch ais_robot_localization localization_monitor_test.launch' C-m


# Attach to the Byobu session
byobu attach -t Localization_Session
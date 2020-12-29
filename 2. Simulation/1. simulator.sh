#!/bin/bash


### KILL ANY PREVIOUS SESSIONS OF THE SIMULATOR ###
tmux kill-session -t simulator


### COPY THE GAZEBO LAUNCH FILE TO THE PX4 FIRMWARE ###
rm -f ~/Firmware/launch/gazebo.launch
cp gazebo.launch ~/Firmware/launch/gazebo.launch


### SOURCE ROS ENVIRONMENT ###
cd ~/Firmware
source ~/catkin_ws/devel/setup.bash


### INICIATE A NEW SESSION ###
tmux new -s simulator -d


### SPLIT THE WINDOW IN TWO DIFFERENT PANES ###
tmux split-window -v -p 10


### RUN THE GAZEBO SIMULATOR IN THE UPPER PANE ####
tmux select-pane -t 0
tmux send-keys "source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default" C-m
tmux send-keys "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)" C-m
tmux send-keys "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo" C-m
tmux send-keys "roslaunch px4 gazebo.launch" C-m


### RUN THE EMULATED MOCAP SYSTEM IN THE BOTTOM PANE ###
tmux select-pane -t 1
tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
tmux send-keys "sleep 30s" C-m
tmux send-keys "rosrun mocap mocap_emulator.py" C-m


### ATTACH SESSION ###
tmux attach-session -t simulator

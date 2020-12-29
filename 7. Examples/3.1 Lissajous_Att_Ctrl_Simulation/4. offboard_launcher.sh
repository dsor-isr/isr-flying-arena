#!/bin/bash



### ARRAYS INITIALIZATION  ###
declare -a ns=()
declare -a fcu_url=()
declare -a ID=()
declare -a mass=()
declare -a radius=()
declare -a height=()
declare -a num_rotors=()
declare -a thrust_curve=()

declare -a comm_library=()
declare -a program_name=()
declare -a program_args=()
declare -a optitrack_pose_forwarder=()
declare -a neighborhood=()
declare -a offboard_logs=()
declare -a real_time_monitor=()
declare -a visualization_tool=()








### STATE THE NUMBER OF DRONES ###
declare -i num_drones=1


### UAV1 ###
ns+=("uav1")
fcu_url+=("udp://:16001@")
ID+=("1")
mass+=("1.52")
radius+=("0.275")
height+=("0.100")
num_rotors+=("4")
thrust_curve+=("iris")

comm_library+=("mavros")
program_name+=("lissajous_att_ctrl.py")
program_args+=("4 4 0.15 4 4 0.5")
optitrack_pose_forwarder+=("")
neighborhood+=("")
offboard_logs+=("yes")
real_time_monitor+=("att")
visualization_tool+=("python")






























### LAUNCH PROGRAM ###


### KILL ANY PREVIOUS SESSIONS ###
tmux kill-session -t connection


### SOURCE ROS ENVIRONMENT ###
cd ~/Firmware
source ~/catkin_ws/devel/setup.bash





### INICIATE A SESSION AND DIVIDE THE WINDOW 0 (ROS LAUNCHERS) IN THE NUMBER OF REQUIRED PANES ###
tmux new -s connection -d
tmux split-window -h -p 50
tmux select-pane -t 0
declare -i aux=0
aux=$(($num_drones/2 + $num_drones%2))
for ((i=1; i<aux; i++)); do
	tmux split-window -v -p $((100 - (100 / num_drones)))
done
tmux select-pane -t $aux
for ((i=1; i<$num_drones/2; i++)); do
	tmux split-window -v -p $((100 - (100 / num_drones)))
done


### CREATE AND DIVIDE THE WINDOW 1 (OPTITRACK FORWARDER) IN THE NUMBER OF REQUIRED PANES ###
tmux new-window -n optitrack_forwarder
tmux select-window -t 1
tmux split-window -v -p 90
tmux select-pane -t 1
tmux split-window -h -p 50
declare -i num_real_drones=0
declare -a real_drones_index=()
declare -i aux2=0
for ((i=0; i<num_drones; i++)); do
	if [ "${optitrack_pose_forwarder[i]}" != "" ] && [ "${optitrack_pose_forwarder[i]}" != "no" ]; then
		num_real_drones=$num_real_drones+1
		real_drones_index+=($i)
	fi
done
aux=$(($num_real_drones/2 + $num_real_drones%2))
tmux select-pane -t 1
for ((i=1; i<aux; i++)); do
	tmux split-window -v -p $((100 - (100 / num_real_drones)))
done
aux2=$(($aux+1))
tmux select-pane -t $aux2
for ((i=1; i<$num_real_drones/2; i++)); do
	tmux split-window -v -p $((100 - (100 / num_real_drones)))
done


###  CREATE AND DIVIDE THE WINDOW 2 (EMULATED SENSORS) IN THE NUMBER OF REQUIRED PANES ###
tmux new-window -n emu_sensors
tmux select-window -t 2
tmux split-window -h -p 50
declare -i num_emu_sensors=0
declare -a emu_sensors_index=()
for ((i=0; i<num_drones; i++)); do
	if [ "${neighborhood[i]}" != "" ] && [ "${neighborhood[i]}" != "no" ]; then
		num_emu_sensors=$num_emu_sensors+1
		emu_sensors_index+=($i)
	fi
done
aux=$(($num_emu_sensors/2 + $num_emu_sensors%2))
tmux select-pane -t 0
for ((i=1; i<aux; i++)); do
	tmux split-window -v -p $((100 - (100 / num_emu_sensors)))
done
tmux select-pane -t $aux
for ((i=1; i<$num_emu_sensors/2; i++)); do
	tmux split-window -v -p $((100 - (100 / num_emu_sensors)))
done


###  CREATE WINDOW 3 (OFFBOARD LOGS) ###
tmux new-window -n offboard_logs


###  CREATE AND DIVIDE THE WINDOW 4 (REAL TIME MONITORS) IN THE NUMBER OF REQUIRED PANES ###
tmux new-window -n monitors
tmux select-window -t 4
tmux split-window -h -p 50
declare -i num_monitors=0
declare -a monitors_index=()
for ((i=0; i<num_drones; i++)); do
	if [ "${real_time_monitor[i]}" != "" ] && [ "${real_time_monitor[i]}" != "no" ]; then
		num_monitors=$num_monitors+1
		monitors_index+=($i)
	fi
done
aux=$(($num_monitors/2 + $num_monitors%2))
tmux select-pane -t 0
for ((i=1; i<aux; i++)); do
	tmux split-window -v -p $((100 - (100 / num_monitors)))
done
tmux select-pane -t $aux
for ((i=1; i<$num_monitors/2; i++)); do
	tmux split-window -v -p $((100 - (100 / num_monitors)))
done


###  CREATE WINDOW 5 (VISUALIZATION TOOL) ###
tmux new-window -n visualization_tool


###  CREATE AND DIVIDE THE WINDOW 6 (OFFBOARD PROGRAMS) IN THE NUMBER OF REQUIRED PANES ###
tmux new-window -n user_programs
tmux select-window -t 6
tmux split-window -h -p 50
declare -i num_user_programs=0
declare -a user_programs_index=()
for ((i=0; i<num_drones; i++)); do
	if [[ "${program_name[i]}" != "" ]]; then
		num_user_programs=$num_user_programs+1
		user_programs_index+=($i)
	fi
done
aux=$(($num_user_programs/2 + $num_user_programs%2))
tmux select-pane -t 0
for ((i=1; i<aux; i++)); do
	tmux split-window -v -p $((100 - (100 / num_user_programs)))
done
tmux select-pane -t $aux
for ((i=1; i<$num_user_programs/2; i++)); do
	tmux split-window -v -p $((100 - (100 / num_user_programs)))
done





### IN WINDOW 0 LAUNCH A ROS NODE FOR EACH DRONE ####
tmux select-window -t 0
tmux select-pane -t 0
tmux send-keys "export ROS_NAMESPACE=${ns[0]}" C-m
tmux send-keys "roslaunch mavros px4.launch fcu_url:=${fcu_url[0]} tgt_system:=${ID[0]}" C-m
tmux send-keys "sleep 5s" C-m
for ((i=1; i<$num_drones; i++)); do
	tmux select-pane -t $i
	tmux send-keys "export ROS_NAMESPACE=${ns[i]}" C-m
	tmux send-keys "roslaunch mavros px4.launch fcu_url:=${fcu_url[i]} tgt_system:=${ID[i]}" C-m
	tmux send-keys "sleep 5s" C-m
done


### IN WINDOW 1 LAUNCH A VPRN_CLIENT AND ROUTE THE POSE TO THE CORRECT TOPICS ####
tmux select-window -t 1
tmux select-pane -t 0
tmux send-keys "roslaunch vrpn_client_ros sample.launch server:=192.168.1.100" C-m
for ((i=0; i<num_real_drones; i++)); do
	aux2=$(($i+1))
	tmux select-pane -t $aux2
	tmux send-keys "sleep 5s" C-m
	tmux send-keys "rosrun topic_tools drop /vrpn_client_node/${optitrack_pose_forwarder[real_drones_index[i]]}/pose 0 1 ${ns[real_drones_index[i]]}/mavros/high_freq_vision_pose/pose" C-m
done
for ((i=0; i<num_real_drones; i++)); do
	if [[ "${thrust_curve[real_drones_index[i]]}" == "snapdragon" ]]; then
		tmux split-window -h -p 50
		aux2=$(($num_real_drones+1))
		tmux select-pane -t $aux2
		tmux send-keys "sleep 5s" C-m
		tmux send-keys "rosrun topic_tools drop /vrpn_client_node/${optitrack_pose_forwarder[real_drones_index[i]]}/pose 2 3 ${ns[real_drones_index[i]]}/mavros/vision_pose/pose" C-m
	fi
done


### IN WINDOW 2 LAUNCH AS MANY INSTANCES OF THE EMULATED SENSORS PROGRAM AS REQUIRED ####
tmux select-window -t 2
for ((i=0; i<num_emu_sensors; i++)); do
	tmux select-pane -t ${i}
	tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
	tmux send-keys "sleep 5s" C-m
	tmux send-keys "rosrun tools rel_pos_vel_emu_sensor.py ${ns[emu_sensors_index[i]]} ${neighborhood[emu_sensors_index[i]]}" C-m
done


### IN WINDOW 3 LAUNCH THE OFFBOARD LOGGER ####
declare -a ns_logs=()
tmux select-window -t 3
tmux select-pane -t 0
for ((i=0; i<num_drones; i++)); do
	if [[ "${offboard_logs[i]}" == "yes" ]]; then
		ns_logs+=(${ns[i]})
	fi
done
tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
tmux send-keys "sleep 10s" C-m
tmux send-keys "rosrun tools offboard_logger.py ${ns_logs[*]}" C-m


### IN WINDOW 4 LAUNCH THE REAL TIME MONITORS ####
tmux select-window -t 4
tmux select-pane -t 0
for ((i=0; i<num_monitors; i++)); do
	tmux select-pane -t ${i}
	tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
	tmux send-keys "sleep 10s" C-m
	tmux send-keys "rosrun tools real_time_monitor.py ${ns[monitors_index[i]]} ${real_time_monitor[monitors_index[i]]}" C-m
done


### IN WINDOW 5 LAUNCH THE VISUALIZATION TOOL ####
declare -a ns_vis=()
tmux select-window -t 5
tmux select-pane -t 0
for ((i=0; i<num_drones; i++)); do
	if [ "${visualization_tool[i]}" != "" ] && [ "${visualization_tool[i]}" != "no" ]; then
		ns_vis+=(${ns[i]})
	fi
done
if [[ "${visualization_tool[0]}" == "python" ]]; then
	tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
	tmux send-keys "sleep 10s" C-m
	tmux send-keys "rosrun tools visualization_tool.py ${ns_vis[*]}" C-m
elif [[ "${visualization_tool[0]}" == "gazebo" ]]; then
	tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
	tmux send-keys "sleep 10s" C-m
	tmux send-keys "rosrun tools gazebo_live_vis_tool.py ${ns_vis[*]}" C-m
fi


### IN WINDOW 6 LAUNCH THE OFFBOARD PROGRAMS ####
tmux select-window -t 6
for ((i=0; i<num_user_programs; i++)); do
	tmux select-pane -t ${i}
	if [[ "${comm_library[user_programs_index[i]]}" == "mavros" ]]; then
		if [[ "${program_name[${user_programs_index[i]}]:(-2)}" == "py" ]]; then
			tmux send-keys "source ~/catkin_ws_python/devel/setup.bash" C-m
			tmux send-keys "sleep 45s" C-m
			tmux send-keys "rosrun offboard_programs ${program_name[${user_programs_index[i]}]} ${ns[user_programs_index[i]]} ${mass[user_programs_index[i]]} ${radius[user_programs_index[i]]}  ${height[user_programs_index[i]]}  ${num_rotors[user_programs_index[i]]}  ${thrust_curve[user_programs_index[i]]} ${program_args[user_programs_index[i]]}" C-m
		elif [[ "${program_name[${user_programs_index[i]}]:(-3)}" == "cpp" ]]; then
			tmux send-keys "source ~/catkin_ws/devel/setup.bash" C-m
			tmux send-keys "sleep 45s" C-m
			tmux send-keys "rosrun ${program_name[${user_programs_index[i]}]::-9} ${program_name[${user_programs_index[i]}]::-4} ${ns[user_programs_index[i]]} ${mass[user_programs_index[i]]} ${radius[user_programs_index[i]]}  ${height[user_programs_index[i]]}  ${num_rotors[user_programs_index[i]]}  ${thrust_curve[user_programs_index[i]]} ${program_args[user_programs_index[i]]}" C-m
		fi

	elif [[ "${comm_library[user_programs_index[i]]}" == "mavsdk" ]]; then
		if [[ "${program_name[${user_programs_index[i]}]:(-2)}" == "py" ]]; then
			tmux send-keys "cd ~/MAVSDK-PYTHON" C-m
			tmux send-keys "sleep 45s" C-m
			tmux send-keys "python3 ${program_name[${user_programs_index[i]}]} $((${fcu_url[${user_programs_index[i]}]:(-6):-1}+6000)) ${mass[user_programs_index[i]]} ${radius[user_programs_index[i]]}  ${height[user_programs_index[i]]}  ${num_rotors[user_programs_index[i]]}  ${thrust_curve[user_programs_index[i]]} ${program_args[user_programs_index[i]]}" C-m
		elif [[ "${program_name[${user_programs_index[i]}]:(-3)}" == "cpp" ]]; then
			tmux send-keys "cd ~/MAVSDK-C++/build" C-m
			tmux send-keys "sleep 45s" C-m
			tmux send-keys "./${program_name[${user_programs_index[i]}]::-4} $((${fcu_url[${user_programs_index[i]}]:(-6):-1}+6000)) ${mass[user_programs_index[i]]} ${radius[user_programs_index[i]]}  ${height[user_programs_index[i]]}  ${num_rotors[user_programs_index[i]]}  ${thrust_curve[user_programs_index[i]]} ${program_args[user_programs_index[i]]}" C-m
		fi
	fi
done





### ATTACH SESSION ###
tmux attach-session -t connection

#!/usr/bin/env python3
"""
Software program that reproduces, through the log files and in a Gazebo window, the evolution of the position and attitude of the drones of the experiment.

This Gazebo-based visualization tool employs the graphical user interface of Gazebo in a lightweight way, that is, significantly reducing the frequency in which vehicles are plotted and eliminating unnecessary animations, such as the motors rotation.
"""





# Import python libraries.
import os
import sys
import math
import yaml
import rospy
import threading
import numpy as np





# Import ROS messages.
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState





def set_model_state(freq, log, vehicle, ID):
	"""
	Sets, in the Gazebo window and using the appropriate ROS service, the position and attitude of the model that represents the drone.

	Parameters
	----------
	freq : float
		Plotting frequency of the vehicle.
	logs : str
		Name of the flight log of the vehicle.
	vehicles : str
		Name of the model of the vehicle.
	IDs : int
		ID of the vehicle.
	"""
	rospy.wait_for_service('/gazebo/set_model_state')
	client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

	rate = rospy.Rate(freq)

	state_msg = ModelState()
	state_msg.model_name = vehicle + '_' + str(ID)

	info = np.genfromtxt(log, skip_header=1, delimiter=',')
	pos_north = list(info[:,1])
	pos_east = list(info[:,2])
	pos_down = list(info[:,3])
	att_q_w = list(info[:,10])
	att_q_x = list(info[:,11])
	att_q_y = list(info[:,12])
	att_q_z = list(info[:,13])

	while(True):
		state_msg.pose.position.x = pos_east.pop(0)
		state_msg.pose.position.y = pos_north.pop(0)
		state_msg.pose.position.z = -pos_down.pop(0)
		state_msg.pose.orientation.x = att_q_y.pop(0)
		state_msg.pose.orientation.y = att_q_x.pop(0)
		state_msg.pose.orientation.z = -att_q_z.pop(0)
		state_msg.pose.orientation.w = att_q_w.pop(0)
		result = client(state_msg)
		rate.sleep()





def read_yaml_configuration_file():
	"""
	Retrieves, from the yaml configuration file associated with this tool, the properties desired by the user for the 3D representation of the drones.

	Returns
	-------
	freq
		Plotting frequency of the vehicles.
	logs
		List with the name of the flight logs of each vehicle.
	vehicles
		List with the name of the model of each vehicle.
	IDs
		List with the ID of each vehicle.
	"""
	logs, vehicles, IDs = [], [], []

	with open(os.path.expanduser('~/catkin_ws_python/src/tools/src/gazebo_logs_vis_tool.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		freq = configs_dict['freq']
		for i in range(1, 500):
			try:
				logs.append(configs_dict['log_'+str(i)])
				vehicles.append(configs_dict['vehicle_'+str(i)])
				IDs.append(configs_dict['ID_'+str(i)])
			except:
				break

	return freq, logs, vehicles, IDs





if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Reads the yaml configuration file associated with this tool. 
	Starts, for each drone, a thread that reads the flight log and reproduces the evolution of the vehicle 3D position and attitude in a Gazebo window.
	"""
	rospy.init_node('gazebo_logs_visualization_tool', anonymous=True)
	
	freq, logs, vehicles, IDs = read_yaml_configuration_file()
		
	threads = []
	for i in range(len(logs)):
		threads.append(threading.Thread(target=set_model_state, args=(freq, logs[i], vehicles[i], IDs[i])))

	for i in range(len(logs)):
		threads[i].start()

	for i in range(len(logs)):
		threads[i].join()

#!/usr/bin/env python3
"""
Software program that reproduces, during an experiment and in a Gazebo window, the evolution of the 3D position and the attitude of the drones.

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
from geometry_msgs.msg import PoseStamped





class GAZEBO_LIVE_VISUALIZATION_TOOL():
	"""
	Class responsible for updating, in a Gazebo window, the position and attitude of a drone.

	Methods
	-------
	init_pose_updates(self)
		Subscribes to the ROS topics with the position and attitude of the drone.
	update_position(self, msg)
		Updates the variable that stores the position of the drone provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude(self, msg)
		Updates the variable that stores the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	set_model_state(self)
		Sets, in the Gazebo window and using the appropriate ROS service, the position and attitude of the model that represents the drone.
	"""
	def __init__(self, drone_ns, freq, vehicle, ID):
		"""
		Constructor of GAZEBO_LIVE_VISUALIZATION_TOOL class. 

		Starts a thread that subscribes to the ROS topics with the position and attitude of the vehicle.
		Calls the method that continuously updates the pose of the drone in the Gazebo window.

		Parameters
		----------
		drone_ns : str
			ROS namespace where the data from the vehicle is encapsulated.
		freq : float
			Updating frequency of the pose of the vehicle.
		vehicle : str
			Name of the model of the vehicle.
		ID : int
			ID of the vehicle.
		"""
		self.ID = ID
		self.freq = freq
		self.vehicle = vehicle
		self.drone_ns = drone_ns

		self.pos = None
		self.att_q = None
	
		threading.Thread(target=self.init_pose_updates, daemon=True).start()
		rate = rospy.Rate(1); rate.sleep()

		self.set_model_state()





	def init_pose_updates(self):
		"""
		Subscribes to the ROS topics with the position and attitude of the drone.
		"""
		pos_sub = rospy.Subscriber(self.drone_ns+'/mavros/local_position/pose', PoseStamped, self.update_position)
		att_sub = rospy.Subscriber(self.drone_ns+'/mavros/local_position/pose', PoseStamped, self.update_attitude)
		rospy.spin()





	def update_position(self, msg):
		"""
		Updates the variable that stores the position of the drone provided by the extended Kalman filter of the PX4 autopilot.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the extended Kalman filter output value for the position of the drone.
		"""
		self.pos = msg





	def update_attitude(self, msg):
		"""
		Updates the variable that stores the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the extended Kalman filter output value for the attitude of the vehicle.
		"""
		self.att_q = msg





	def set_model_state(self):
		"""
		Sets, in the Gazebo window and using the appropriate ROS service, the position and attitude of the model that represents the drone.
		"""
		rospy.wait_for_service('/gazebo/set_model_state')
		client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		rate = rospy.Rate(freq)

		state_msg = ModelState()
		state_msg.model_name = self.vehicle + '_' + str(self.ID)

		while(True):
			state_msg.pose.position = self.pos.pose.position
			state_msg.pose.orientation = self.att_q.pose.orientation
			result = client(state_msg)
			rate.sleep()





def read_yaml_configuration_file():
	"""
	Retrieves, from the yaml configuration file associated with this tool, the properties desired by the user for the 3D representation of the drones.

	Returns
	-------
	freq
		Updating frequency of the pose of the vehicles.
	vehicles
		List with the name of the model of each vehicle.
	IDs
		List with the ID of each vehicle.
	"""
	vehicles, IDs = [], []

	with open(os.path.expanduser('~/catkin_ws_python/src/tools/src/gazebo_live_vis_tool.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		freq = configs_dict['freq']
		for i in range(1, 500):
			try:
				vehicles.append(configs_dict['vehicle_'+str(i)])
				IDs.append(configs_dict['ID_'+str(i)])
			except:
				break

	return freq, vehicles, IDs





if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Reads the yaml configuration file associated with this tool. 
	Starts, for each drone, an object of the GAZEBO_LIVE_VISUALIZATION_TOOL class that updates the vehicle position and attitude in a Gazebo window.
	"""
	rospy.init_node('gazebo_live_visualization_tool', anonymous=True)
	
	freq, vehicles, IDs = read_yaml_configuration_file()
		
	for i in range(len(sys.argv[1:])):
		GAZEBO_LIVE_VISUALIZATION_TOOL(sys.argv[1:][i], freq, vehicles[i], IDs[i])

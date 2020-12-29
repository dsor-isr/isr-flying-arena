#!/usr/bin/env python3
"""
Software program that emulates a relative position sensor and a relative velocity sensor.

The program receives, as command line arguments, the namespace of the drone that "carries" the two sensors and the namespace of each one of its neighbour vehicles.

The relative position and velocity measurements are obtained by subtracting the position and velocity of each one of the drones of the experiment to the position and velocity of the vehicle with the sensors, and by adding adjustable Gaussian white noise to the results.
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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray




# Global dictionaries declaration.
pos = {}
vel = {}




def read_yaml_configuration_file():
	"""
	Retrieves, from the yaml configuration file associated with this tool, the standard deviation of the white Gaussian noise that will be added to the computed relative positions and velocities. This function also retrieves, from the same yaml file, the publishing frequency of the emulated measurements.

	Returns
	-------
	pub_freq
		Publishing frequency of the ROS messages containing the emulated measurements of the relative position and velocity of each one of the neighbour vehicles of the drone.
	pos_std_dev
		Standard deviation, expressed in meters, of the white Gaussian noise that will be added to the computed relative positions.
	vel_std_dev
		Standard deviation, expressed in meters per second, of the white Gaussian noise that will be added to the computed relative velocities.
	"""
	with open(os.path.expanduser('~/catkin_ws_python/src/tools/src/rel_pos_vel_emu_sensor.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		pub_freq = configs_dict['publishing_frequency']
		pos_std_dev = configs_dict['position_standard_deviation']
		vel_std_dev = configs_dict['velocity_standard_deviation']

	return pub_freq, pos_std_dev, vel_std_dev




def publisher():
	"""
	Publishes, in custom ROS topics, the emulated measurements of the relative position and velocity of the neighbour vehicles of the drone.

	Adds white Gaussian noise to the relative position and velocity values obtained, in order to represent the uncertainty of the measurements provided by real sensors.
	"""
	global pos, vel

	pub_freq, pos_std_dev, vel_std_dev = read_yaml_configuration_file()
	rate = rospy.Rate(pub_freq)

	pos_pub = rospy.Publisher(sys.argv[1]+'/mavros/relative_positions/list', Float64MultiArray, queue_size=1)
	vel_pub = rospy.Publisher(sys.argv[1]+'/mavros/relative_velocities/list', Float64MultiArray, queue_size=1)

	position = Float64MultiArray()
	velocity = Float64MultiArray()

	while(True):
		data=[]
		pos_drone = pos[sys.argv[1]]
		for ns in sys.argv[2:]:
			pos_neigh = pos[ns]
			noise = np.random.normal(0, pos_std_dev, 3)
			data.extend([pos_neigh[0][0]-pos_drone[0][0]+noise[0], pos_neigh[1][0]-pos_drone[1][0]+noise[1], pos_neigh[2][0]-pos_drone[2][0]+noise[2]])
		position.data = data
		pos_pub.publish(position)

		data=[]
		vel_drone = vel[sys.argv[1]]
		for ns in sys.argv[2:]:
			vel_neigh = vel[ns]
			noise = np.random.normal(0, vel_std_dev, 3)
			data.extend([vel_neigh[0][0]-vel_drone[0][0]+noise[0], vel_neigh[1][0]-vel_drone[1][0]+noise[1], vel_neigh[2][0]-vel_drone[2][0]+noise[2]])
		velocity.data = data
		vel_pub.publish(velocity)

		rate.sleep()




def enu_to_ned(v):
	"""
	Converts a 3x1 physical variable from ENU coordinates to NED coordinates.

	Parameters
	----------
	v : np.array of floats with shape (3,1)
		3x1 physical variable in ENU coordinates.

	Returns
	-------
	np.array of floats with shape (3,1)
		3x1 physical variable in NED coordinates.
	"""
	return np.array([[v[1][0]],[v[0][0]],[-v[2][0]]])




def update_vel(msg, key):
	"""
	Updates the dictionary that stores the velocity of the vehicles. 

	Parameters
	----------
	msg : ModelStates (from gazebo_msgs)
		ROS message containing the velocity of a vehicle.
	"""
	global vel

	vel[key] = enu_to_ned(np.array([[msg.twist.linear.x], [msg.twist.linear.y], [msg.twist.linear.z]]))




def update_pos(msg, key):
	"""
	Updates the dictionary that stores the position of the vehicles. 

	Parameters
	----------
	msg : ModelStates (from gazebo_msgs)
		ROS message containing the pose of a vehicle.
	"""
	global pos

	pos[key] = enu_to_ned(np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))




def subscriber():
	"""
	Subscribes to the PX4 topics with the position and velocity information of the drone and of each one of its neighbour vehicles.
	Defines the functions 'update_pos' and 'update_vel' as the handlers of these subscriptions. 
	"""	
	for ns in sys.argv[1:]:
		pos_sub = rospy.Subscriber(ns+'/mavros/local_position/pose', PoseStamped, update_pos, ns)
		vel_sub = rospy.Subscriber(ns+'/mavros/local_position/velocity_local', TwistStamped, update_vel, ns)

	rospy.spin()




if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Starts a thread responsible for subscribing to the PX4 topics that contain the position and velocity of the drone and of each one of its neighbour vehicles.
	Creates a thread that publishes, in ROS topics, the emulated measurements of the relative position and velocity of the neighbour vehicles of the drone.
	"""
	rospy.init_node('rel_sensor_'+sys.argv[1], anonymous=True)

	threading.Thread(target=subscriber, daemon=True).start()
	rate=rospy.Rate(0.5); rate.sleep()

	thread=threading.Thread(target=publisher)

	thread.start(); thread.join()

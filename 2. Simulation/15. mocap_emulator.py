#!/usr/bin/env python3





# Import python libraries.
import os
import math
import yaml
import rospy
import threading
import numpy as np





# Import ROS messages.
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion





# Global variables declaration.
ros_id = None
ros_pose = None





def add_white_gaussian_noise(ros_pose, pos_std_dev, att_std_dev):
	"""
	Adds white Gaussian noise to the true position and attitude values retrieved from the Gazebo simulator, in order to represent the uncertainty of the measurements provided by a real motion capture system.

	Parameters
	----------
	ros_pose : PoseStamped (from geometry_msgs)
		ROS message containing the true position and attitude of the vehicle. 
	pos_std_dev: float
		Standard deviation, expressed in meters, of the white Gaussian noise added to the true position of the vehicle.
	att_std_dev : float
		Standard deviation, expressed in degrees, of the white Gaussian noise added to the true attitude of the vehicle.

	Returns
	-------
	ros_pose
		ROS message containing the emulated noisy measurements of the position and attitude of the vehicle. 
	"""
	noisy_ros_pose = ros_pose

	noisy_ros_pose.position.x += np.random.normal(0, pos_std_dev) 
	noisy_ros_pose.position.y += np.random.normal(0, pos_std_dev) 
	noisy_ros_pose.position.z += np.random.normal(0, pos_std_dev)

	ros_q = [noisy_ros_pose.orientation.x, noisy_ros_pose.orientation.y, noisy_ros_pose.orientation.z, noisy_ros_pose.orientation.w]
	noisy_ros_pose.orientation = Quaternion(*quaternion_from_euler(*np.radians(np.degrees(euler_from_quaternion(ros_q))+np.random.normal(0, att_std_dev, 3))))
 	
	return noisy_ros_pose





def pose_publisher(drone_id, drone_ns, num_drones):
	"""	
	Sends, to the PX4 autopilot and to the user programs, noisy measurements of the pose of the vehicle with the ID and namespace specified in the drone_id and drone_ns arguments.

	Parameters
	----------
	drone_id : str
		ID of the drone.
	drone_ns: str
		Namespace of the drone.
	num_drones : int
		Number of simulated drones.
	"""
	with open(os.path.expanduser('~/catkin_ws_python/src/mocap/src/mocap_config.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		frequency = configs_dict['frequency']
		pos_std_dev = configs_dict['position_standard_deviation']
		att_std_dev = configs_dict['attitude_standard_deviation']

	rate = rospy.Rate(frequency)
	pose_pub = rospy.Publisher(drone_ns+'/mavros/vision_pose/pose', PoseStamped, queue_size=1)
	pose_pub_usr = rospy.Publisher(drone_ns+'/mavros/high_freq_vision_pose/pose', PoseStamped, queue_size=1)

	pose = PoseStamped()
	pose.header = Header()

	for i in range(1, num_drones+1):
		if drone_id in ros_id[i]:
			break

	print('\nMocap Emulation started for the drone with the namespace: ' + drone_ns)

	counter=0
	while(1):
		pose.header.stamp = rospy.Time.now()
		pose.pose = add_white_gaussian_noise(ros_pose[i], pos_std_dev, att_std_dev)
		if counter==3: pose_pub.publish(pose); counter=0 
		pose_pub_usr.publish(pose)
		rate.sleep(); counter+=1





def update_pose(msg):
	"""
	Updates the variables that store the ID and pose of all vehicles. 

	Parameters
	----------
	msg : ModelStates (from gazebo_msgs)
		ROS message containing the name and pose of all vehicles.
	"""
	global ros_pose, ros_id
	ros_id = msg.name
	ros_pose = msg.pose





def gazebo_subscriber():
	"""
	Subscribes to the gazebo topic with the pose information of all vehicles.
	Defines the function 'update_pose' as the handler of this subscription. 
	"""
	pose_sub = rospy.Subscriber('gazebo/model_states', ModelStates, update_pose)
	rospy.spin()





def get_vehicles_id_ns():
	"""
	Gets, from the gazebo.launch file, the ID and namespace of each simulated drone, returning this information in a dictionary.

	Returns
	-------
	dictionary
		Each item represents a drone: the key is the drone ID and the value is the drone namespace.
	"""
	dict_id_ns = {}
	file = open("launch/gazebo.launch", "r")
	for line in file:
		if "group ns" in line:
			ns = line[15:-3]
		if "ID" in line and "LAUNCH" not in line:
			v_id = line[34:-4]
			dict_id_ns[v_id] = ns
	return dict_id_ns





if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Gets the ID and namespace selected by the user for each simulated drone.
	Starts the thread responsible for subscribing to the gazebo topic that contains the pose of all vehicles.
	For each vehicle, creates a thread that sends the pose data to the PX4 autopilot.
	"""
	rospy.init_node('mocap_emulator', anonymous=True)
	
	vehicles_id_ns = get_vehicles_id_ns()

	threading.Thread(target=gazebo_subscriber, daemon=True).start()
	rate = rospy.Rate(0.5); rate.sleep()

	num_drones = len(vehicles_id_ns)
	threads_pubs = []

	for key in vehicles_id_ns:
		threads_pubs.append(threading.Thread(target=pose_publisher, args=(key, vehicles_id_ns[key], num_drones)))

	for i in range(num_drones):
		threads_pubs[i].start()

	for i in range(num_drones):
		threads_pubs[i].join()

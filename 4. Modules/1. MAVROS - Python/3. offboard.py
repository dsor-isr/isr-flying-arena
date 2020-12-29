#!/usr/bin/env python3





# Import python libraries
import math
import rospy
import threading
import numpy as np
from scipy.linalg import norm





# Import ROS messages
from std_msgs.msg import Header
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import ActuatorControl
from tf.transformations import quaternion_from_euler





class OFFBOARD:
	"""
	Stores the methods that send offboard commands and offboard control references to the PX4 autopilot of the vehicle.

	Methods
	-------
	arm_drone(self)
		Arms the drone, if it is not already armed.
	start_offboard_mode(self)
		Changes the flight mode of the PX4 autopilot of the drone to offboard.
	start_offboard_mission(self)
		Makes the vehicle ready for an offboard experiment by arming it and by changing the flight mode of its PX4 autopilot to offboard.
	set_pos_yaw(self, pos, yaw, time)
		Offboard method that sends position and yaw references to the PX4 autopilot of the the drone.
	set_vel_yaw(self, vel, yaw, freq)
		Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.
	set_vel_body_yaw_rate(self, vel_body, yaw_rate, freq):
		Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.
	set_att_thrust(self, att, att_type, thrust, freq)
		Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.
	set_ang_vel_thrust(self, ang_vel, thrust, freq):
		Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.
	set_act(self, group, output, freq)
		Offboard method that sets the values of the mixers and/or actuators of the vehicle.
	disarm_drone(self)
		Disarms the vehicle, if it is not already disarmed.
	auto_land(self)
		Lands the drone, changing its flight mode to auto-land.
	"""
	def arm_drone(self):
		"""
		Arms the drone, if it is not already armed.

		Makes a request, to the ROS service responsible for changing the armed state of the vehicle, to switch the arm value to True.
		"""
		if self.info.is_armed == False:
			rospy.wait_for_service(self.info.drone_ns + '/mavros/cmd/arming')
			arm_client = rospy.ServiceProxy(self.info.drone_ns + '/mavros/cmd/arming', CommandBool)
			arm = True
			print('\nArming drone...')
			result = arm_client(arm)
			rate = rospy.Rate(1); rate.sleep()
			print('Drone armed!') if result.success and self.info.is_armed == True else print('Unable to arm the drone.')
		else:
			print('\nThe drone is already armed!')





	def start_offboard_mode(self):
		"""
		Changes the flight mode of the PX4 autopilot of the drone to offboard.

		Makes a request, to the ROS service responsible for changing the flight mode, to switch it to OFFBOARD.
		"""
		if self.info.flight_mode != 'OFFBOARD':
			rospy.wait_for_service(self.info.drone_ns + '/mavros/set_mode')
			set_offboard_client = rospy.ServiceProxy(self.info.drone_ns + '/mavros/set_mode', SetMode)
			self.set_pos_yaw(self.ekf.pos, self.ekf.att_euler[2][0], 1)
			print('\nChanging to offboard mode...')
			result = set_offboard_client(0, 'OFFBOARD')
			rate=rospy.Rate(1); rate.sleep()
			print('Offboard mode activated!') if result.mode_sent else print('Unable to change to offboard mode.')
		else:
			print('\nThe offboard mode is already set!')





	def start_offboard_mission(self):
		"""
		Makes the vehicle ready for an offboard experiment by arming it and by changing the flight mode of its PX4 autopilot to offboard.
		"""
		self.start_offboard_mode()
		self.arm_drone()





	def set_pos_yaw(self, pos, yaw, time):
		"""
		Offboard method that sends position and yaw references to the PX4 autopilot of the the drone.

		Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS.

		Parameters
		----------
		pos : np.array of floats with shape (3,1)
			Desired position for the drone, in local NED coordinates.
		yaw : float
			Desired yaw for the vehicle, in radians.
		time: float
			Time, in seconds, during which the selected position and yaw references will be sent to the PX4 autopilot.
		"""
		t0 = rospy.get_time()
		t = rospy.get_time()-t0
		while(t < time):
			pub = rospy.Publisher(self.info.drone_ns + '/mavros/setpoint_position/local', PoseStamped, queue_size=1)
			msg = PoseStamped()
			msg.header = Header()
			msg.header.stamp = rospy.Time.now()
			msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = ned_to_enu(pos)
			msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, -yaw))
			pub.publish(msg)
			rate=rospy.Rate(50); rate.sleep()
			t = rospy.get_time()-t0





	def set_vel_yaw(self, vel, yaw, freq):
		"""
		Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.

		Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS.

		Parameters
		----------
		vel : np.array of floats with shape (3,1)
			Desired linear velocity for drone, in local NED coordinates.
		yaw : float
			Desired yaw for the vehicle, in radians.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		pub = rospy.Publisher(self.info.drone_ns + '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
		msg = PositionTarget()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 1; msg.type_mask = 3015
		msg.velocity.x, msg.velocity.y, msg.velocity.z = ned_to_enu(vel)
		msg.yaw = -yaw
		pub.publish(msg)
		rate=rospy.Rate(freq); rate.sleep()





	def set_vel_body_yaw_rate(self, vel_body, yaw_rate, freq):
		"""
		Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.

		Makes convertions from the body NED frame adopted for the ISR Flying Arena to the body ENU frame used by ROS.

		Parameters
		----------
		vel_body : np.array of floats with shape (3,1)
			Desired linear velocity for the drone, in body NED coordinates.
		yaw_rate : float
			Desired yaw rate for the vehicle, in radians per second.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		pub = rospy.Publisher(self.info.drone_ns + '/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
		msg = PositionTarget()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8; msg.type_mask = 1991
		msg.velocity.x, msg.velocity.y, msg.velocity.z = ned_to_enu(vel_body)
		msg.yaw_rate = -yaw_rate
		pub.publish(msg)
		rate=rospy.Rate(freq); rate.sleep()





	def set_att_thrust(self, att, att_type, thrust, freq):
		"""
		Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.

		Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS. 
		Converts the thrust from newtons to a normalized value between 0 and 1 through mathematical expression of the thrust curve of the vehicle.

		Parameters
		----------
		att : np.array of floats with shape (3,1) or with shape (4,1)
			Desired attitude for the vehicle, expressed in euler angles or in a quaternion.
		att_type : str
			Must be equal to either 'euler' or 'quaternion'. Specifies the format of the desired attitude.
		thrust : float
			Desired thrust value in newtons.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		pub = rospy.Publisher(self.info.drone_ns + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
		msg = AttitudeTarget()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()
		if att_type == 'euler':
			msg.orientation = Quaternion(*quaternion_from_euler(*ned_to_enu(att)))
		elif att_type == 'quaternion':
			msg.orientation = Quaternion(*SI_quaternion_to_ROS_quaternion(att))				
		msg.thrust = normalize_thrust(thrust, self.info.thrust_curve, norm(self.ekf.vel))
		pub.publish(msg)
		rate=rospy.Rate(freq); rate.sleep()





	def set_ang_vel_thrust(self, ang_vel, thrust, freq):
		"""
		Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.

		Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS.
		Converts the thrust from newtons to a normalized value between 0 and 1 through the mathematical expression of the thrust curve of the vehicle.

		Parameters
		----------
		ang_vel : np.array of floats with shape (3,1)
			Desired angular velocity for the drone.
		thrust : float
			Desired thrust value in newtons.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		pub = rospy.Publisher(self.info.drone_ns + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
		msg = AttitudeTarget()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now(); msg.type_mask = 128
		msg.body_rate.x, msg.body_rate.y, msg.body_rate.z = ned_to_enu(ang_vel)
		msg.thrust = normalize_thrust(thrust, self.info.thrust_curve, norm(self.ekf.vel))
		pub.publish(msg)
		rate=rospy.Rate(freq); rate.sleep()





	def set_act(self, group, output, freq):
		"""
		Offboard method that sets the values of the mixers and/or actuators of the vehicle.

		Parameters
		----------
		group: int
			Desired control group.
		output : list
			Desired output values for the mixers and/or actuators of the drone.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		pub = rospy.Publisher(self.info.drone_ns + '/mavros/actuator_control', ActuatorControl, queue_size=1)
		msg = ActuatorControl()
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()
		msg.group_mix = group
		msg.controls = output
		pub.publish(msg)
		rate=rospy.Rate(freq); rate.sleep()





	def disarm_drone(self):
		"""
		Disarms the vehicle, if it is not already disarmed.

		Makes a request, to the ROS service responsible for changing the armed state of the vehicle, to switch the arm value to False.
		"""
		if self.info.is_armed == True:
			rospy.wait_for_service(self.info.drone_ns + '/mavros/cmd/arming')
			disarm_client = rospy.ServiceProxy(self.info.drone_ns + '/mavros/cmd/arming', CommandBool)
			arm = False
			print('\nDisarming drone...')
			result = disarm_client(arm)
			rate=rospy.Rate(1); rate.sleep()
			print('Drone disarmed!') if result.success and self.info.is_armed == False else print('Unable to disarm the drone.')
		else:
			print('\nThe drone is already disarmed!')





	def auto_land(self):
		"""
		Lands the drone, changing its flight mode to auto-land.
		"""
		if self.info.flight_mode != 'AUTO.LAND':
			rospy.wait_for_service(self.info.drone_ns + '/mavros/set_mode')
			set_auto_land_client = rospy.ServiceProxy(self.info.drone_ns+'/mavros/set_mode', SetMode)
			print('\nChanging to auto-land mode...')
			result = set_auto_land_client(0, 'AUTO.LAND')
			rate=rospy.Rate(1); rate.sleep()
			print('Auto-land mode activated!') if result.mode_sent and self.info.flight_mode == 'AUTO.LAND' else print('Unable to change to auto-land mode.')
		else:
			print('\nThe drone is already in auto-land mode!')





def ned_to_enu(v):
	"""
	Converts a 3x1 physical variable from NED coordinates to ENU coordinates.

	Parameters
	----------
	v : np.array of floats with shape (3,1)
		3x1 physical variable in NED coordinates.

	Returns
	-------
	np.array of floats with shape (3,1)
		3x1 physical variable in ENU coordinates.
	"""
	return np.array([[v[1][0]],[v[0][0]],[-v[2][0]]])





def SI_quaternion_to_ROS_quaternion(q):
	"""
	Converts a SI quaternion in NED coordinates to a ROS quaternion in ENU coordinates.

	Parameters
	----------
	q: np.array of floats with shape (4,1)
		Quaternion in the SI format and NED coordinates.

	Returns
	-------
	list
		Quaternion in the ROS format and ENU coordinates.
	"""
	return [q[2][0],q[1][0],-q[3][0],q[0][0]]





def normalize_thrust(thrust_newtons, thrust_curve, vel):
	"""
	Converts the thrust in newtons to a normalized value between 0 and 1 through the mathematical expression of the thrust curve of the drone.

	Parameters
	----------
	thrust_newtons : float
		Desired thrust value in newtons.
	thrust_curve: str
		Thrust curve of the vehicle.
	vel : float
		Norm of the linear velocity of the vehicle.
	
	Returns
	-------
	float
		Normalized thrust, between 0 and 1.
	"""
	if thrust_newtons <= 0:
		return 0.0
	elif thrust_curve == 'iris':
		norm_thrust = (thrust_newtons/(1-vel/25) - 1.52*9.80665)/(2*34.068*0.561 + 7.1202) + 0.561 
	elif thrust_curve == 'intel_aero':
		norm_thrust = (np.tan((thrust_newtons-10.37)/8.84)+1.478)/2.955
	elif thrust_curve == 'snapdragon':
		norm_thrust = 0.1377*np.exp(0.02976*thrust_newtons)*np.sqrt(thrust_newtons) + 0.003759*thrust_newtons - 0.05973

	return 0.0 if norm_thrust <= 0 else 1.0 if norm_thrust >= 1 else norm_thrust	

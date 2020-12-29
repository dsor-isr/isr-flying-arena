#!/usr/bin/env python3





# Import python libraries
import math
import rospy
import numpy as np





# Import ROS messages
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ExtendedState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import ActuatorControl
from tf.transformations import euler_from_quaternion





class TELEMETRY:
	"""
	Stores the methods responsible for keeping the variables of the UAV class up to date.

	Methods
	-------
	init_telemetry(self)
		Manages topic subscriptions.
	update_position(self, msg)
		Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity(self, msg)
		Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity_body(self, msg)
		Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude(self, msg)
		Updates the variables that store the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_angular_velocity(self, msg)
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_imu(self, msg)
		Updates the variables that store the linear acceleration and the angular velocity of the drone measured by the IMU.
	update_mag(self, msg)
		Updates the variable that stores the magnetic field vector, in body NED coordinates, measured by the IMU.
	update_mocap(self, msg)
		Updates the variables that store the position and attitude of the drone provided by the motion capture system.
	update_gps(self, msg)
		Updates the variable that stores the raw measurements provided by the GPS sensor.
	update_baro_pressure(self, msg)
		Updates the variable that stores the static pressure measured by the barometer.
	update_baro_temperature(self, msg)
		Updates the variable that stores the temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.
	update_baro_altitude(self, msg)
		Updates the variable that stores the altitude of the vehicle, above mean sea level, computed through the barometric atmospheric pressure and the temperature.
	update_relative_positions(self, msg)
		Updates the variable that stores the relative position of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative position sensor.
	update_relative_velocities(self, msg)
		Updates the variable that stores the relative velocity of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative velocity sensor.
	update_actuator(self, msg)
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
	update_status(self, msg)
		Updates the variables that store the current flight mode of the PX4 autopilot, the system status, and the armed state of the vehicle. 
	update_landed(self, msg)
		Updates the variable that stores the landed state of the drone.
	update_battery(self, msg)
		Updates the variable that stores the remaining battery percentage. 
	"""
	def init_telemetry(self):
		"""
		Manages topic subscriptions. 

		Sets the topics subscribers and defines the methods responsible for handling each subscription.
		"""
		pos_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/local_position/pose', PoseStamped, self.update_position)
		vel_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/local_position/velocity_local', TwistStamped, self.update_velocity)
		vel_body_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/local_position/velocity_body', TwistStamped, self.update_velocity_body)
		att_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/local_position/pose', PoseStamped, self.update_attitude)
		ang_vel_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/local_position/velocity_body', TwistStamped, self.update_angular_velocity)
		imu_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/imu/data', Imu, self.update_imu)
		mag_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/imu/mag', MagneticField, self.update_mag)
		mocap_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/high_freq_vision_pose/pose', PoseStamped, self.update_mocap)
		gps_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/global_position/raw/fix', NavSatFix, self.update_gps)
		baro_pressure_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/imu/static_pressure', FluidPressure, self.update_baro_pressure)
		baro_temperature_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/imu/temperature_imu', Temperature, self.update_baro_temperature)
		baro_altitude_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/altitude', Altitude, self.update_baro_altitude)
		rel_pos_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/relative_positions/list', Float64MultiArray, self.update_relative_positions)
		rel_vel_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/relative_velocities/list', Float64MultiArray, self.update_relative_velocities)
		act_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/target_actuator_control', ActuatorControl, self.update_actuator)
		status_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/state', State, self.update_status)
		landed_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/extended_state', ExtendedState, self.update_landed)
		battery_sub = rospy.Subscriber(self.info.drone_ns+'/mavros/battery', BatteryState, self.update_battery)
		rospy.spin()





	def update_position(self, msg):
		"""
		Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the local NED frame since the ROS message stores the position of the drone in the local ENU frame.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the extended Kalman filter output value for the position of the drone.
		"""
		self.ekf.pos = enu_to_ned(np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))





	def update_velocity(self, msg):
		"""
		Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the local NED frame since the ROS message stores the velocity of the drone in the local ENU frame.

		Parameters
		----------
		msg : TwistStamped (from geometry_msgs)
			ROS message containing the extended Kalman filter output value for the velocity of the drone.
		"""
		self.ekf.vel = enu_to_ned(np.array([[msg.twist.linear.x], [msg.twist.linear.y], [msg.twist.linear.z]]))





	def update_velocity_body(self, msg):
		"""
		Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the body NED frame since the ROS message stores the velocity of the drone in the body ENU frame.

		Parameters
		----------
		msg : TwistStamped (from geometry_msgs)
			ROS message containing the extended Kalman filter output value for the velocity of the drone.
		"""
		self.ekf.vel_body = enu_to_ned(np.array([[msg.twist.linear.x], [msg.twist.linear.y], [msg.twist.linear.z]]))





	def update_attitude(self, msg):
		"""
		Updates the variables that store the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to local NED frame since the ROS message stores the attitude of the drone in the local ENU frame.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the extended Kalman filter output value for the attitude of the vehicle.
		"""
		ros_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
		self.ekf.att_q = ROS_quaternion_to_SI_quaternion(ros_q)
		self.ekf.att_euler = enu_to_ned(np.asarray(euler_from_quaternion(ros_q)).reshape(3,1))





	def update_angular_velocity(self, msg):
		"""
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the local NED frame since the ROS message stores the angular velocity of the drone in the local ENU frame.

		Parameters
		----------
		msg : TwistStamped (from geometry_msgs)
			ROS message containing the the extended Kalman filter output value for the angular velocity of the vehicle.
		"""
		self.ekf.ang_vel = enu_to_ned(np.array([[msg.twist.angular.x], [msg.twist.angular.y], [msg.twist.angular.z]]))





	def update_imu(self, msg):
		"""
		Updates the variables that store the linear acceleration and the angular velocity of the drone measured by the IMU.

		Makes appropriate convertions to the body NED frame since the ROS message stores the IMU data in the body ENU frame.

		Parameters
		----------
		msg : Imu (from sensor_msgs)
			ROS message containing the IMU measurements for the linear acceleration and angular velocity of the vehicle.
		"""
		self.sen.imu.acc_body = enu_to_ned(np.array([[msg.linear_acceleration.x], [msg.linear_acceleration.y], [msg.linear_acceleration.z]]))
		self.sen.imu.ang_vel = enu_to_ned(np.array([[msg.angular_velocity.x], [msg.angular_velocity.y], [msg.angular_velocity.z]]))





	def update_mag(self, msg):
		"""
		Updates the variable that stores the magnetic field vector, in body NED coordinates, measured by the IMU.

		Makes appropriate convertions to the body NED frame since the ROS message stores the magnetic field vector in the body ENU frame.

		Parameters
		----------
		msg : MagneticField (from sensor_msgs)
			ROS message containing the IMU measurements for the magnetic field vector.
		"""
		self.sen.imu.mag = enu_to_ned(np.array([[msg.magnetic_field.x], [msg.magnetic_field.y], [msg.magnetic_field.z]]))





	def update_mocap(self, msg):
		"""
		Updates the variables that store the position and attitude of the drone provided by the motion capture system.

		Makes appropriate convertions to the body NED frame since the ROS message stores the MOCAP pose in the body ENU frame.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs)
			ROS message containing the pose of the vehicle provided by the motion capture system.
		"""
		self.sen.mocap.pos = enu_to_ned(np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))
		ros_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
		self.sen.mocap.att_q = ROS_quaternion_to_SI_quaternion(ros_q)
		self.sen.mocap.att_euler = enu_to_ned(np.asarray(euler_from_quaternion(ros_q)).reshape(3,1))





	def update_gps(self, msg):
		"""
		Updates the variable that stores the raw measurements provided by the GPS sensor.

		Makes appropriate convertions to SI units since the ROS message stores the latitude and longitude in degrees.

		Parameters
		----------
		msg : NavSatFix (from sensors_msgs)
			ROS message containing the GPS sensor measurements for the position of the vehicle in gps coordinates.
		"""
		self.sen.gps.pos = np.array([[math.radians(msg.latitude)], [math.radians(msg.longitude)], [msg.altitude]])





	def update_baro_pressure(self, msg):
		"""
		Updates the variable that stores the static pressure measured by the barometer.

		Parameters
		----------
		msg : FluidPressure (from sensors_msgs)
			ROS message containing the static pressure measurements provided by the barometer.
		"""
		self.sen.baro.pressure = msg.fluid_pressure





	def update_baro_temperature(self, msg):
		"""
		Updates the variable that stores the temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.

		Parameters
		----------
		msg : Temperature (from sensors_msgs)
			ROS message containing the temperature measurements provided by the thermometer integrated in the barometer.
		"""
		self.sen.baro.temperature = msg.temperature





	def update_baro_altitude(self, msg):
		"""
		Updates the variable that stores the altitude of the vehicle, above mean sea level, computed through the barometric atmospheric pressure and the temperature.

		Parameters
		----------
		msg : Altitude (from mavros_msgs)
			ROS message containing the altitude values computed through the barometric atmospheric pressure and the temperature.
		"""
		self.sen.baro.alt = msg.monotonic





	def update_relative_positions(self, msg):
		"""
		Updates the variable that stores the relative position of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative position sensor.

		Parameters
		----------
		msg : Float64MultiArray (from std_msgs) 
			ROS message containing a list of the relative position of each of the n neighbour vehicles, in local NED coordinates.
		"""
		self.sen.emu.rel_pos = ROS_list_to_np_array(msg.data)





	def update_relative_velocities(self, msg):
		"""
		Updates the variable that stores the relative velocity of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative velocity sensor.

		Parameters
		----------
		msg : Float64MultiArray (from std_msgs) 
			ROS message containing a list of the relative velocity of each of the n neighbour vehicles, in local NED coordinates.
		"""	
		self.sen.emu.rel_vel = ROS_list_to_np_array(msg.data)





	def update_actuator(self, msg):
		"""
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.

		Parameters
		----------
		msg : ActuatorControl (from mavros_msgs) 
			ROS message containing the normalized values applied to the mixer and/or motors and servos of the vehicle.
		"""
		self.act.group = msg.group_mix
		self.act.output = msg.controls





	def update_status(self, msg):
		"""
		Updates the variables that store the current flight mode of the PX4 autopilot, the system status, and the armed state of the vehicle. 

		Parameters
		----------
		msg : State (from mavros_msgs) 
			ROS message containing general information about the status of the drone and of the PX4 autopilot.
		"""
		self.info.flight_mode = msg.mode
		self.info.is_connected = msg.connected
		self.info.is_armed = msg.armed





	def update_landed(self, msg):
		"""
		Updates the variable that stores the landed state of the drone.

		Parameters
		----------
		msg : ExtendedState (from mavros_msgs) 
			ROS message containing information about the landed state of the drone.
		"""
		self.info.is_landed = msg.landed_state == 1





	def update_battery(self, msg):
		"""
		Updates the variable that stores the remaining battery percentage. 

		Parameters
		----------
		msg : BatteryState (from sensor_msgs) 
			ROS message containing information about the battery of the drone.
		"""
		self.info.battery = msg.percentage*100





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





def ROS_quaternion_to_SI_quaternion(q):
	"""
	Converts a ROS quaternion in ENU coordinates to a SI quaternion in NED coordinates.

	Parameters
	----------
	q : list
		Quaternion in the ROS format and ENU coordinates.

	Returns
	-------
	np.array of floats with shape (4,1)
		Quaternion in the SI format and NED coordinates.
	"""
	return np.array([[q[3]],[q[1]],[q[0]],[-q[2]]])





def ROS_list_to_np_array(msg_data):
	"""
	Converts a one dimensional ROS message with 3*n elements into a numpy array of shape (n,3,1).

	Parameters
	----------
	msg_data : Float64MultiArray (from std_msgs) 
		One dimensional ROS message with 3*n elements containing a list of the relative position or velocity of each of the n neighbour vehicles, in NED coordinates.

	Returns
	-------
	np.array of floats with shape (n,3,1)
		Relative position or velocity of each of the n neighbour vehicles, in NED coordinates.
	"""
	one_dim_list=[]

	for i in range(1000):
		try:
			one_dim_list.append(msg_data[i])
		except:
			break		

	return np.reshape(one_dim_list, (i//3,3,1))

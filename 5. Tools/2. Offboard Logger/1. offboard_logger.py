#!/usr/bin/env python3
"""
Software program that subscribes to all the information related to the state of a set of vehicles and logs it into CSV files. 
This program also produces plots with the time evolution of the logged physical quantities.

This tool enables the rapid validation of control algorithms because the CSV file and the plots become immediately available to the user and allow the comparison of the actual and the desired values of a physical quantity.
"""





#Import python libraries.
import os
import sys
import math
import yaml
import rospy
import datetime
import threading
import numpy as np
import pandas as pd
from uav_mavros import UAV
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)





# Import ROS messages.
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import AttitudeTarget





class OFFBOARD_LOGGER():
	"""
	Class responsible for logging all the information related to the state of an UAV into a CSV file and for generating the plots of the logged physical quantities.

	Methods
	-------
	start_logging_sequence(self)
		Runs the logging sequence.
	log_parameters(self)
		Logs all the physical quantities related to the state of the UAV into dictionaries.
	ref_subscriber(self)
		Subscribes to the ROS topics with the desired references for the position, attitude and linear and angular velocities of the drone.
	update_position_ref(self, msg)
		Updates the variable that stores the desired position for the vehicle. 
	update_velocity_ref(self, msg)
		Updates the variable that stores the desired velocity for the vehicle. 
	update_attitude_ref(self, msg)
		Updates the variable that stores the desired attitude for the vehicle. 
	update_ang_velocity_ref(self, msg)
		Updates the variable that stores the desired angular velocity for the vehicle. 
	generate_csv(self)
		Saves all the logged physical quantities related to the state of the UAV into a CSV file.
	plot_logs(self)
		Plots the time evolution of the most important logged physical quantities.
	single_plot(self, x, y, y_ref_name, y_lim, title, label_x, label_y, fig_size, img_name)
		Plots a set of data.
	"""
	def __init__(self, drone_ns, log_freq, date, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref):
		"""
		Constructor of the OFFBOARD_LOGGER class. 

		Contains dictionaries to store the data of each physical quantity, since that is more efficient to add elements to dictionaries than to append elements to large lists.

		Parameters
		----------
		drone_ns : str
			ROS namespace where the data from the PX4 autopilot and the MOCAP system is encapsulated.
		log_freq : float
			Offboard logging frequency.
		date : str
			Date of the start of the logged flight.
		source_pos_ref : str
			States the source of the position control references. 
		source_vel_ref : str
			States the source of the velocity control references. 
		source_att_ref : str
			States the source of the attitude control references. 
		source_ang_vel_ref : str
			States the source of the angular velocity control references. 
		"""
		self.drone_ns = drone_ns
		self.log_freq = log_freq
		self.date = date
		self.source_pos_ref = source_pos_ref
		self.source_vel_ref = source_vel_ref
		self.source_att_ref = source_att_ref
		self.source_ang_vel_ref = source_ang_vel_ref
		self.df = None

		self.pos_ref = None; self.vel_ref = None; self.att_euler_ref = None; self.ang_vel_ref = None
		self.pos_ref_flag = False; self.vel_ref_flag = False; self.att_euler_ref_flag = False; self.ang_vel_ref_flag = False
		self.dict_ekf_pos_north, self.dict_ekf_pos_east, self.dict_ekf_pos_down = {}, {}, {}
		self.dict_ekf_vel_north, self.dict_ekf_vel_east, self.dict_ekf_vel_down = {}, {}, {} 
		self.dict_ekf_vel_body_north, self.dict_ekf_vel_body_east, self.dict_ekf_vel_body_down = {}, {}, {}
		self.dict_ekf_att_q_w, self.dict_ekf_att_q_x, self.dict_ekf_att_q_y, self.dict_ekf_att_q_z = {}, {}, {}, {}
		self.dict_ekf_att_euler_roll, self.dict_ekf_att_euler_pitch, self.dict_ekf_att_euler_yaw = {}, {}, {}
		self.dict_ekf_ang_vel_roll, self.dict_ekf_ang_vel_pitch, self.dict_ekf_ang_vel_yaw = {}, {}, {}
		self.dict_imu_acc_body_north, self.dict_imu_acc_body_east, self.dict_imu_acc_body_down = {}, {}, {} 
		self.dict_imu_ang_vel_roll, self.dict_imu_ang_vel_pitch, self.dict_imu_ang_vel_yaw = {}, {}, {}
		self.dict_imu_mag_north, self.dict_imu_mag_east, self.dict_imu_mag_down = {}, {}, {}
		self.dict_mocap_pos_north, self.dict_mocap_pos_east, self.dict_mocap_pos_down = {}, {}, {} 
		self.dict_mocap_att_q_w, self.dict_mocap_att_q_x, self.dict_mocap_att_q_y, self.dict_mocap_att_q_z = {}, {}, {} , {}
		self.dict_mocap_att_euler_roll, self.dict_mocap_att_euler_pitch, self.dict_mocap_att_euler_yaw = {}, {}, {}
		self.dict_flight_mode, self.dict_is_armed, self.dict_is_landed, self.dict_battery = {}, {}, {}, {}
		self.dict_pos_ref_north, self.dict_pos_ref_east, self.dict_pos_ref_down = {}, {}, {}
		self.dict_vel_ref_north, self.dict_vel_ref_east, self.dict_vel_ref_down = {}, {}, {} 
		self.dict_att_euler_ref_roll, self.dict_att_euler_ref_pitch, self.dict_att_euler_ref_yaw = {}, {}, {}
		self.dict_ang_vel_ref_roll, self.dict_ang_vel_ref_pitch, self.dict_ang_vel_ref_yaw = {}, {}, {}






	def start_logging_sequence(self):
		"""
		Runs the logging sequence.

		Calls the method that starts logging all the information related to the vehicle state and, once the flight is finished, calls the methods that generate the CSV file and the plots of the logged physical quantities.
		"""
		self.log_parameters()
		self.generate_csv()
		self.plot_logs()






	def log_parameters(self):
		"""
		Logs all the physical quantities related to the state of the UAV into dictionaries.

		Dictionaries are used because it is more efficient to add items to them than to append items to large lists.

		This method subscribes to the information related to the state of a vehicle using the methods of the TELEMETRY class.
		"""
		uav = UAV(self.drone_ns, 0, 0, 0, 0, None)
		threading.Thread(target=self.ref_subscriber, daemon=True).start()

		rate = rospy.Rate(self.log_freq)
		
		while(uav.info.is_armed == False):
			rate.sleep()

		while(uav.info.is_armed == True):
		
			time = rospy.get_time()

			self.dict_ekf_pos_north[time] = uav.ekf.pos[0][0]
			self.dict_ekf_pos_east[time] = uav.ekf.pos[1][0]
			self.dict_ekf_pos_down[time] = uav.ekf.pos[2][0]

			self.dict_ekf_vel_north[time] = uav.ekf.vel[0][0]
			self.dict_ekf_vel_east[time] = uav.ekf.vel[1][0]
			self.dict_ekf_vel_down[time] = uav.ekf.vel[2][0]

			self.dict_ekf_vel_body_north[time] = uav.ekf.vel_body[0][0]
			self.dict_ekf_vel_body_east[time] = uav.ekf.vel_body[1][0]
			self.dict_ekf_vel_body_down[time] = uav.ekf.vel_body[2][0]

			self.dict_ekf_att_q_w[time] = uav.ekf.att_q[0][0]
			self.dict_ekf_att_q_x[time] = uav.ekf.att_q[1][0]
			self.dict_ekf_att_q_y[time] = uav.ekf.att_q[2][0]
			self.dict_ekf_att_q_z[time] = uav.ekf.att_q[3][0]

			self.dict_ekf_att_euler_roll[time] = uav.ekf.att_euler[0][0]
			self.dict_ekf_att_euler_pitch[time] = uav.ekf.att_euler[1][0]
			self.dict_ekf_att_euler_yaw[time] = uav.ekf.att_euler[2][0]

			self.dict_ekf_ang_vel_roll[time] = uav.ekf.ang_vel[0][0]
			self.dict_ekf_ang_vel_pitch[time] = uav.ekf.ang_vel[1][0]
			self.dict_ekf_ang_vel_yaw[time] = uav.ekf.ang_vel[2][0]

			self.dict_imu_acc_body_north[time] = uav.sen.imu.acc_body[0][0]
			self.dict_imu_acc_body_east[time] = uav.sen.imu.acc_body[1][0]
			self.dict_imu_acc_body_down[time] = uav.sen.imu.acc_body[2][0]

			self.dict_imu_ang_vel_roll[time] = uav.sen.imu.ang_vel[0][0]
			self.dict_imu_ang_vel_pitch[time] = uav.sen.imu.ang_vel[1][0]
			self.dict_imu_ang_vel_yaw[time] = uav.sen.imu.ang_vel[2][0]

			self.dict_imu_mag_north[time] = uav.sen.imu.ang_vel[0][0]
			self.dict_imu_mag_east[time] = uav.sen.imu.ang_vel[1][0]
			self.dict_imu_mag_down[time] = uav.sen.imu.ang_vel[2][0]

			self.dict_mocap_pos_north[time] = uav.sen.mocap.pos[0][0]
			self.dict_mocap_pos_east[time] = uav.sen.mocap.pos[1][0]
			self.dict_mocap_pos_down[time] = uav.sen.mocap.pos[2][0]

			self.dict_mocap_att_q_w[time] = uav.sen.mocap.att_q[0][0]
			self.dict_mocap_att_q_x[time] = uav.sen.mocap.att_q[1][0]
			self.dict_mocap_att_q_y[time] = uav.sen.mocap.att_q[2][0]
			self.dict_mocap_att_q_z[time] = uav.sen.mocap.att_q[3][0]

			self.dict_mocap_att_euler_roll[time] = uav.ekf.att_euler[0][0]
			self.dict_mocap_att_euler_pitch[time] = uav.ekf.att_euler[1][0]
			self.dict_mocap_att_euler_yaw[time] = uav.ekf.att_euler[2][0]

			self.dict_flight_mode[time] = uav.info.flight_mode
			self.dict_is_armed[time] = uav.info.is_armed
			self.dict_is_landed[time] = uav.info.is_landed
			self.dict_battery[time] = uav.info.battery

			if self.pos_ref_flag != False:
				self.dict_pos_ref_north[time] = self.pos_ref[0][0]
				self.dict_pos_ref_east[time] = self.pos_ref[1][0]
				self.dict_pos_ref_down[time] = self.pos_ref[2][0]

			if self.vel_ref_flag != False:
				self.dict_vel_ref_north[time] = self.vel_ref[0][0]
				self.dict_vel_ref_east[time] = self.vel_ref[1][0]
				self.dict_vel_ref_down[time] = self.vel_ref[2][0]

			if self.att_euler_ref_flag != False:
				self.dict_att_euler_ref_roll[time] = self.att_euler_ref[0][0]
				self.dict_att_euler_ref_pitch[time] = self.att_euler_ref[1][0]
				self.dict_att_euler_ref_yaw[time] = self.att_euler_ref[2][0]

			if self.ang_vel_ref_flag != False:
				self.dict_ang_vel_ref_roll[time] = self.ang_vel_ref[0][0]
				self.dict_ang_vel_ref_pitch[time] = self.ang_vel_ref[1][0]
				self.dict_ang_vel_ref_yaw[time] = self.ang_vel_ref[2][0]

			rate.sleep()






	def ref_subscriber(self):
		"""
		Subscribes to the ROS topics with the desired references for the position, attitude and linear and angular velocities of the drone.
		"""
		if self.source_pos_ref == 'user':
			pos_ref_sub= rospy.Subscriber(self.drone_ns+'/mavros/waypoints/position', PoseStamped, self.update_position_ref)
		elif self.source_pos_ref == 'px4':
			pos_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_position/local', PoseStamped, self.update_position_ref)

		if self.source_vel_ref == 'user':
			vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/waypoints/velocity', PositionTarget, self.update_velocity_ref)
		elif self.source_vel_ref == 'px4':
			vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_raw/local', PositionTarget, self.update_velocity_ref)

		if self.source_att_ref == 'user':
			att_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/waypoints/attitude', AttitudeTarget, self.update_attitude_ref)
		elif self.source_att_ref == 'px4':
			att_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_raw/attitude', AttitudeTarget, self.update_attitude_ref)

		if self.source_ang_vel_ref == 'user':
			ang_vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/waypoints/angular_velocity', AttitudeTarget, self.update_ang_velocity_ref)
		elif self.source_ang_vel_ref == 'px4':
			ang_vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_raw/attitude', AttitudeTarget, self.update_ang_velocity_ref)






	def update_position_ref(self, msg):
		"""
		Updates the variable that stores the desired position for the vehicle. 

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the desired position for the vehicle.
		"""
		self.pos_ref = enu_to_ned(np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))
		self.pos_ref_flag = True






	def update_velocity_ref(self, msg):
		"""
		Updates the variable that stores the desired velocity for the vehicle. 

		Parameters
		----------
		msg : PositionTarget (from mavros_msgs)
			ROS message containing the desired velocity for the vehicle.
		"""
		self.vel_ref = enu_to_ned(np.array([[msg.velocity.x], [msg.velocity.y], [msg.velocity.z]]))
		self.vel_ref_flag = True






	def update_attitude_ref(self, msg):
		"""
		Updates the variable that stores the desired attitude for the vehicle. 

		Parameters
		----------
		msg : AttitudeTarget (from mavros_msgs)
			ROS message containing the desired attitude for the vehicle.
		"""
		ros_q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
		self.att_euler_ref = enu_to_ned(np.asarray(euler_from_quaternion(ros_q)).reshape(3,1))
		self.att_euler_ref_flag = True






	def update_ang_velocity_ref(self, msg):
		"""
		Updates the variable that stores the desired angular velocity for the vehicle. 

		Parameters
		----------
		msg : AttitudeTarget (from mavros_msgs)
			ROS message containing the desired angular velocity for the vehicle.
		"""
		self.ang_vel_ref = enu_to_ned(np.array([[msg.body_rate.x], [msg.body_rate.y], [msg.body_rate.z]]))
		self.ang_vel_ref_flag = True






	def generate_csv(self):
		"""
		Saves all the logged physical quantities related to the state of the UAV into a CSV file.

		Creates a pandas dataframe from the set of dictionaries that store all the logged information.

		Exports the created pandas dataframe to a CSV file.
		"""
		self.df = pd.DataFrame.from_dict(self.dict_ekf_pos_north.items())
		self.df = self.df.rename(columns = {0:'time.seconds', 1:'ekf.pos.north'})

		self.df['ekf.pos.east'] = self.df['time.seconds'].map(self.dict_ekf_pos_east)
		self.df['ekf.pos.down'] = self.df['time.seconds'].map(self.dict_ekf_pos_down)
		self.df['ekf.vel.north'] = self.df['time.seconds'].map(self.dict_ekf_vel_north)
		self.df['ekf.vel.east'] = self.df['time.seconds'].map(self.dict_ekf_vel_east)
		self.df['ekf.vel.down'] = self.df['time.seconds'].map(self.dict_ekf_vel_down)
		self.df['ekf.vel_body.north'] = self.df['time.seconds'].map(self.dict_ekf_vel_body_north)
		self.df['ekf.vel_body.east'] = self.df['time.seconds'].map(self.dict_ekf_vel_body_east)
		self.df['ekf.vel_body.down'] = self.df['time.seconds'].map(self.dict_ekf_vel_body_down)
		self.df['ekf.att_q.w'] = self.df['time.seconds'].map(self.dict_ekf_att_q_w)
		self.df['ekf.att_q.x'] = self.df['time.seconds'].map(self.dict_ekf_att_q_x)
		self.df['ekf.att_q.y'] = self.df['time.seconds'].map(self.dict_ekf_att_q_y)
		self.df['ekf.att_q.z'] = self.df['time.seconds'].map(self.dict_ekf_att_q_z)
		self.df['ekf.att_euler.roll'] = self.df['time.seconds'].map(self.dict_ekf_att_euler_roll)
		self.df['ekf.att_euler.pitch'] = self.df['time.seconds'].map(self.dict_ekf_att_euler_pitch)
		self.df['ekf.att_euler.yaw'] = self.df['time.seconds'].map(self.dict_ekf_att_euler_yaw)
		self.df['ekf.ang_vel.roll'] = self.df['time.seconds'].map(self.dict_ekf_ang_vel_roll)
		self.df['ekf.ang_vel.pitch'] = self.df['time.seconds'].map(self.dict_ekf_ang_vel_pitch)
		self.df['ekf.ang_vel.yaw'] = self.df['time.seconds'].map(self.dict_ekf_ang_vel_yaw)
		self.df['imu.acc_body.north'] = self.df['time.seconds'].map(self.dict_imu_acc_body_north)
		self.df['imu.acc_body.east'] = self.df['time.seconds'].map(self.dict_imu_acc_body_east)
		self.df['imu.acc_body.down'] = self.df['time.seconds'].map(self.dict_imu_acc_body_down)
		self.df['imu.ang_vel.roll'] = self.df['time.seconds'].map(self.dict_imu_ang_vel_roll)
		self.df['imu.ang_vel.pitch'] = self.df['time.seconds'].map(self.dict_imu_ang_vel_pitch)
		self.df['imu.ang_vel.yaw'] = self.df['time.seconds'].map(self.dict_imu_ang_vel_yaw)
		self.df['imu.mag.north'] = self.df['time.seconds'].map(self.dict_imu_mag_north)
		self.df['imu.mag.east'] = self.df['time.seconds'].map(self.dict_imu_mag_east)
		self.df['imu.mag.down'] = self.df['time.seconds'].map(self.dict_imu_mag_down)
		self.df['mocap.pos.north'] = self.df['time.seconds'].map(self.dict_mocap_pos_north)
		self.df['mocap.pos.east'] = self.df['time.seconds'].map(self.dict_mocap_pos_east)
		self.df['mocap.pos.down'] = self.df['time.seconds'].map(self.dict_mocap_pos_down)
		self.df['mocap.att_q.w'] = self.df['time.seconds'].map(self.dict_mocap_att_q_w)
		self.df['mocap.att_q.x'] = self.df['time.seconds'].map(self.dict_mocap_att_q_x)
		self.df['mocap.att_q.y'] = self.df['time.seconds'].map(self.dict_mocap_att_q_y)
		self.df['mocap.att_q.z'] = self.df['time.seconds'].map(self.dict_mocap_att_q_z)
		self.df['mocap.att_euler.roll'] = self.df['time.seconds'].map(self.dict_mocap_att_euler_roll)
		self.df['mocap.att_euler.pitch'] = self.df['time.seconds'].map(self.dict_mocap_att_euler_pitch)
		self.df['mocap.att_euler.yaw'] = self.df['time.seconds'].map(self.dict_mocap_att_euler_yaw)
		self.df['info.flight_mode'] = self.df['time.seconds'].map(self.dict_flight_mode)
		self.df['info.is_armed'] = self.df['time.seconds'].map(self.dict_is_armed)
		self.df['info.is_landed'] = self.df['time.seconds'].map(self.dict_is_landed)
		self.df['info.battery'] = self.df['time.seconds'].map(self.dict_battery)

		if self.pos_ref_flag != False:
			self.df['pos_ref.north'] = self.df['time.seconds'].map(self.dict_pos_ref_north)
			self.df['pos_ref.east'] = self.df['time.seconds'].map(self.dict_pos_ref_east)
			self.df['pos_ref.down'] = self.df['time.seconds'].map(self.dict_pos_ref_down)

		if self.vel_ref_flag != False:
			self.df['vel_ref.north'] = self.df['time.seconds'].map(self.dict_vel_ref_north)
			self.df['vel_ref.east'] = self.df['time.seconds'].map(self.dict_vel_ref_east)
			self.df['vel_ref.down'] = self.df['time.seconds'].map(self.dict_vel_ref_down)

		if self.att_euler_ref_flag != False:
			self.df['att_euler_ref.roll'] = self.df['time.seconds'].map(self.dict_att_euler_ref_roll)
			self.df['att_euler_ref.pitch'] = self.df['time.seconds'].map(self.dict_att_euler_ref_pitch)
			self.df['att_euler_ref.yaw'] = self.df['time.seconds'].map(self.dict_att_euler_ref_yaw)

		if self.ang_vel_ref_flag != False:
			self.df['ang_vel_ref.roll'] = self.df['time.seconds'].map(self.dict_ang_vel_ref_roll)
			self.df['ang_vel_ref.pitch'] = self.df['time.seconds'].map(self.dict_ang_vel_ref_pitch)
			self.df['ang_vel_ref.yaw'] = self.df['time.seconds'].map(self.dict_ang_vel_ref_yaw)

		self.df.to_csv(os.path.expanduser('~/catkin_ws_python/src/tools/src/logs/') + self.date + '/' + self.drone_ns + '/' + self.date + '_' + self.drone_ns + '.csv', index=False)






	def plot_logs(self):
		"""
		Plots the time evolution of the most important logged physical quantities.

		Calls the single_plot method in order to generate each individual plot.
		"""
		self.single_plot(self.df['time.seconds'], self.df['ekf.pos.north'], 'pos_ref.north', (-3, 3), self.drone_ns+': Log of the Position North', 'Time [seconds]', 'North [m]', (16,9), self.drone_ns+'_0_ekf_pos_north.png')
		self.single_plot(self.df['time.seconds'], self.df['ekf.pos.east'], 'pos_ref.east', (-2, 2), self.drone_ns+': Log of the Position East', 'Time [seconds]', 'East [m]', (16,9), self.drone_ns+'_1_ekf_pos_east.png')
		self.single_plot(self.df['time.seconds'], self.df['ekf.pos.down']*-1, 'pos_ref.down', (0, 2.5), self.drone_ns+': Log of the -(Position Down)', 'Time [seconds]', '-Down [m]', (16,9), self.drone_ns+'_2_ekf_pos_down.png')
		self.single_plot(self.df['time.seconds'], self.df['ekf.vel.north'], 'vel_ref.north', (-3, 3), self.drone_ns+': Log of the Velocity North', 'Time [seconds]', 'Velocity North [m/s]', (16,9), self.drone_ns+'_3_ekf_vel_north.png')
		self.single_plot(self.df['time.seconds'], self.df['ekf.vel.east'], 'vel_ref.east', (-3, 3), self.drone_ns+': Log of the Velocity East', 'Time [seconds]', 'Velocity East [m/s]', (16,9), self.drone_ns+'_4_ekf_vel_east.png')
		self.single_plot(self.df['time.seconds'], self.df['ekf.vel.down']*-1, 'vel_ref.down', (-3, 3), self.drone_ns+': Log of the -(Velocity Down)', 'Time [seconds]', '-Velocity Down [m/s]', (16,9), self.drone_ns+'_5_ekf_vel_down.png')
		self.single_plot(self.df['time.seconds'], np.degrees(self.df['ekf.att_euler.roll']), 'att_euler_ref.roll', (-45, 45), self.drone_ns+': Log of the Roll Angle', 'Time [seconds]', 'Roll Angle [degrees]', (16,9), self.drone_ns+'_6_ekf_att_euler_roll.png')
		self.single_plot(self.df['time.seconds'], np.degrees(self.df['ekf.att_euler.pitch']), 'att_euler_ref.pitch', (-45, 45), self.drone_ns+': Log of the Pitch Angle', 'Time [seconds]', 'Pitch Angle [degrees]', (16,9), self.drone_ns+'_7_ekf_att_euler_pitch.png')
		self.single_plot(self.df['time.seconds'], np.degrees(self.df['ekf.att_euler.yaw']), 'att_euler_ref.yaw', (-180, 180), self.drone_ns+': Log of the Yaw Angle', 'Time [seconds]', 'Yaw Angle [degrees]', (16,9), self.drone_ns+'_8_ekf_att_euler_yaw.png')
		self.single_plot(self.df['time.seconds'], np.degrees(self.df['ekf.ang_vel.roll']), 'ang_vel_ref.roll', (-180, 180), self.drone_ns+': Log of the Roll Rate', 'Time [seconds]', 'Roll Rate [deg/s]', (16,9), self.drone_ns+'_9_ekf_ang_vel_roll.png')
		self.single_plot(self.df['time.seconds'], np.degrees(self.df['ekf.ang_vel.pitch']), 'ang_vel_ref.pitch', (-180, 180), self.drone_ns+': Log of the Pitch Rate', 'Time [seconds]', 'Pitch Rate [deg/s]', (16,9), self.drone_ns+'_10_ekf_ang_vel_pitch.png')
		self.single_plot(self.df['time.seconds'], np.degrees(self.df['ekf.ang_vel.yaw']), 'ang_vel_ref.yaw', (-180, 180), self.drone_ns+': Log of the Yaw Rate', 'Time [seconds]', 'Yaw Rate [deg/s]', (16,9), self.drone_ns+'_11_ekf_ang_vel_yaw.png')






	def single_plot(self, x, y, y_ref_name, y_lim, title, label_x, label_y, fig_size, img_name):
		"""
		Plots a set of data.

		Parameters
		----------
		x : pandas series
			Set of values for the x axis.
		y : pandas seriea
			Set of values for the y axis.
		y_ref_name : str
			Name of the column of the dataframe that stores the control references of the desired physical variable.
		y_lim : tuple
			Desired limits for the y axis.
		title : str
			Title of the plot.
		label_x : str
			Label of the x axis of the plot.
		label_y : str
			Label of the y axis of the plot.
		fig_size : tuple
			Size of the plot.
		img_name : str
			Name of the file with the plot.
		"""
		fig, ax = plt.subplots(figsize = fig_size)
		plt.subplots_adjust(left = 0.07, bottom = 0.10, right = 0.98, top = 0.92, wspace = None, hspace = None)

		rect = fig.patch
		rect.set_facecolor('lightblue')

		ax.set_facecolor('whitesmoke')
		ax.set_title(title, pad = 10, fontsize = 20, fontweight = 'bold')

		try:
			y_ref = self.df[y_ref_name]
			if 'down' in y_ref_name:
				y_ref *= -1
			if 'att' in y_ref_name or 'ang' in y_ref_name:
				y_ref = np.degrees(y_ref)
			ax.plot(x, y_ref, 'black', label='Reference')
		except:
			pass	
	
		ax.plot(x, y, 'blue', label = 'Real')
		ax.legend(prop = {'size': 12})

		ax.set_xlabel(label_x, labelpad = 8, fontsize = 15, fontweight = 'bold')
		ax.set_ylabel(label_y, labelpad = 5, fontsize = 15, fontweight = 'bold')

		ax.set_xlim(x.iloc[0], x.iloc[-1])
		ax.set_ylim(y_lim)
		ax.tick_params(axis = 'both', which = 'major', labelsize = 10)
		ax.tick_params(axis = 'both', which = 'minor', labelsize = 5)
		ax.xaxis.set_major_locator(MultipleLocator((x.iloc[-1]-x.iloc[0])/12))
		ax.xaxis.set_minor_locator(AutoMinorLocator(5))
		ax.yaxis.set_major_locator(MultipleLocator((y_lim[1]-y_lim[0])/8))
		ax.yaxis.set_minor_locator(AutoMinorLocator(5))
		ax.grid(which='major', color='#000000', linestyle='--')
		ax.grid(which='minor', color='#000000', linestyle=':')

		fig.savefig(os.path.expanduser('~/catkin_ws_python/src/tools/src/logs/') + self.date + '/' + self.drone_ns + '/' + img_name, facecolor=fig.get_facecolor())






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






def read_yaml_configuration_file():
	"""
	Retrieves, from the yaml configuration file associated with this tool, the logging frequency and the sources of the control references.

	Returns
	-------
	log_freq
		Offboard logging frequency.
	source_pos_ref
		States the source of the position control references. 
	source_vel_ref
		States the source of the velocity control references. 
	source_att_ref
		States the source of the attitude control references. 
	source_ang_vel_ref
		States the source of the angular velocity control references. 
	"""
	with open(os.path.expanduser('~/catkin_ws_python/src/tools/src/offboard_logger.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		log_freq = configs_dict['logging_frequency']
		source_pos_ref = configs_dict['source_position_ref']
		source_vel_ref = configs_dict['source_velocity_ref']
		source_att_ref = configs_dict['source_attitude_ref']
		source_ang_vel_ref = configs_dict['source_angular_velocity_ref']

	return log_freq, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref






def create_storing_folders():
	"""
	Creates the folders that will store the generated CSV files and plots.	
		
	The name of the folders depend on the day, hour, and minute of the start of the logged flight.
	"""
	date = datetime.datetime.now()
	date = date.strftime("%Y") + '_' + date.strftime("%m") + '_' + date.strftime("%d") + '_' + date.strftime("%H") + 'h_' + date.strftime("%M")

	try:
		os.mkdir(os.path.expanduser('~/catkin_ws_python/src/tools/src/logs/') + date)
	except FileExistsError:
		pass

	for drone_ns in sys.argv[1:]:
		try:
			os.mkdir(os.path.expanduser('~/catkin_ws_python/src/tools/src/logs/') + date + '/' + drone_ns)
		except FileExistsError:
			pass

	return date






if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Creates folders for storing the generated CSV files and plots. Reads the yaml configuration file associated with this tool.

	Creates, for each vehicle, an instance of the OFFBOARD_LOGGER class and a thread that will run the offboard logging sequence.
	"""
	rospy.init_node('offboard_logger', anonymous=True)

	date = create_storing_folders()
	log_freq, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref = read_yaml_configuration_file()

	logger_objects = []
	for drone_ns in sys.argv[1:]:
		offboard_logger = OFFBOARD_LOGGER(drone_ns, log_freq, date, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref)	
		logger_objects.append(offboard_logger)	

	logger_threads = []
	for offboard_logger in logger_objects:	
		logger_threads.append(threading.Thread(target=offboard_logger.start_logging_sequence))

	for thread in logger_threads:
		thread.start()

	for thread in logger_threads:
		thread.join()

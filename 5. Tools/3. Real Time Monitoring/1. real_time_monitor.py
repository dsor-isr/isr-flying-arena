#!/usr/bin/env python3
"""
Software program that enables the user to monitor the state of the vehicles during experiments.

This tool is important for the developed testing framework because it complements and improves the monitoring setup available in the QGroundControl software, in which the user has to monitor the components of all desired variables in a single window with only two subplots.
"""





#Import python libraries
import os
import sys
import math
import yaml
import rospy
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tf.transformations import euler_from_quaternion
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)





# Import ROS messages.
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import AttitudeTarget





class REAL_TIME_MONITOR():
	"""
	Class responsible for plotting, in a window, the time evolution of the desired and measured control variables.

	Methods
	-------
	subscriber(self)
		Subscribes to the ROS topics with the measured and desired values for the position, attitude and linear and angular velocities of the drone.
	update_position(self, msg)
		Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity(self, msg)
		Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude(self, msg)
		Updates the variables that store the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_angular_velocity(self, msg)
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_position_ref(self, msg)
		Updates the variable that stores the desired position for the vehicle. 
	update_velocity_ref(self, msg)
		Updates the variable that stores the desired velocity for the vehicle. 
	update_attitude_ref(self, msg)
		Updates the variable that stores the desired attitude for the vehicle. 
	update_ang_velocity_ref(self, msg)
		Updates the variable that stores the desired angular velocity for the vehicle. 
	start_window(self)
		Creates a window to display the time evolution of the selected physical quantities.
	static_plot_pos(self)
		Plots the static elements of the position monitoring window.
	static_plot_vel(self)
		Plots the static elements of the velocity monitoring window.
	static_plot_att(self)
		Plots the static elements of the attitude monitoring window.
	static_plot_ang_vel(self)
		Plots the static elements of the angular velocity monitoring window.
	dynamic_plot(self, _)
		Plots the dynamic elements of the monitoring window.
	"""
	def __init__(self, drone_ns, graph_type, update_freq, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref, time_window):
		"""
		Constructor of the REAL_TIME_MONITOR class.

		Starts a thread to subscribe to the ROS topics that store the state of the drone.
		Calls the method that creates a window to display the time evolution of the selected physical quantities. 

		Parameters
		----------
		drone_ns : str
			ROS namespace where the data from the PX4 autopilot and the MOCAP system is encapsulated.
		graph_type : str
			States the control variable to be monitored.
		update_freq : float
			Update frequency of the window.
		source_pos_ref : str
			States the source of the position control references. 
		source_vel_ref : str
			States the source of the velocity control references. 
		source_att_ref : str
			States the source of the attitude control references. 
		source_ang_vel_ref : str
			States the source of the angular velocity control references.
		time_window : float
			Limits of the time axis.
		"""
		self.drone_ns = drone_ns
		self.graph_type = graph_type
		self.update_freq = update_freq
		self.source_pos_ref = source_pos_ref
		self.source_vel_ref = source_vel_ref
		self.source_att_ref = source_att_ref
		self.source_ang_vel_ref = source_ang_vel_ref
		self.time_window = time_window

		self.ekf_pos = None; self.ekf_vel = None; self.ekf_att_euler = None; self.ekf_ang_vel = None
		self.pos_ref = None; self.vel_ref = None; self.att_euler_ref = None; self.ang_vel_ref = None
		self.pos_ref_flag = False; self.vel_ref_flag = False; self.att_euler_ref_flag = False; self.ang_vel_ref_flag = False

		self.x = list(np.arange(-self.time_window, 1/self.update_freq, 1/self.update_freq))
		self.y1 = [0]*(self.time_window*self.update_freq+1); self.y2 = [0]*(self.time_window*self.update_freq+1)
		self.y3 = [0]*(self.time_window*self.update_freq+1); self.y4 = [0]*(self.time_window*self.update_freq+1)
		self.y5 = [0]*(self.time_window*self.update_freq+1); self.y6 = [0]*(self.time_window*self.update_freq+1)

		self.fig = None; self.ax1 = None; self.ax2 = None; self.ax3 = None 
		self.line1 = None; self.line2 = None; self.line3 = None; self.line4 = None; self.line5 = None; self.line6 = None

		threading.Thread(target=self.subscriber, daemon=True).start()

		self.start_window()






	def subscriber(self):
		"""
		Subscribes to the ROS topics with the measured and desired values for the position, attitude and linear and angular velocities of the drone.
		"""
		if self.graph_type == 'pos':
			pos_sub = rospy.Subscriber(self.drone_ns+'/mavros/local_position/pose', PoseStamped, self.update_position)
			if self.source_pos_ref == 'user':
				pos_ref_sub= rospy.Subscriber(self.drone_ns+'/mavros/waypoints/position', PoseStamped, self.update_position_ref)
			elif self.source_pos_ref == 'px4':
				pos_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_position/local', PoseStamped, self.update_position_ref)

		elif self.graph_type == 'vel':
			vel_sub = rospy.Subscriber(self.drone_ns+'/mavros/local_position/velocity_local', TwistStamped, self.update_velocity)
			if self.source_vel_ref == 'user':
				vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/waypoints/velocity', PositionTarget, self.update_velocity_ref)
			elif self.source_vel_ref == 'px4':
				vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_raw/local', PositionTarget, self.update_velocity_ref)

		elif self.graph_type == 'att':
			att_sub = rospy.Subscriber(self.drone_ns+'/mavros/local_position/pose', PoseStamped, self.update_attitude)
			if self.source_att_ref == 'user':
				att_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/waypoints/attitude', AttitudeTarget, self.update_attitude_ref)
			elif self.source_att_ref == 'px4':
				att_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_raw/attitude', AttitudeTarget, self.update_attitude_ref)

		elif self.graph_type == 'ang_vel':
			ang_vel_sub = rospy.Subscriber(self.drone_ns+'/mavros/local_position/velocity_body', TwistStamped, self.update_angular_velocity)
			if self.source_ang_vel_ref == 'user':
				ang_vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/waypoints/angular_velocity', AttitudeTarget, self.update_ang_velocity_ref)
			elif self.ang_vel_ref == 'px4':
				ang_vel_ref_sub = rospy.Subscriber(self.drone_ns+'/mavros/setpoint_raw/attitude', AttitudeTarget, self.update_ang_velocity_ref)






	def update_position(self, msg):
		"""
		Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the local NED frame since the ROS message stores the position of the drone in the local ENU frame.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the extended Kalman filter output value for the position of the drone.
		"""
		self.ekf_pos = enu_to_ned(np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))






	def update_velocity(self, msg):
		"""
		Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the local NED frame since the ROS message stores the velocity of the drone in the local ENU frame.

		Parameters
		----------
		msg : TwistStamped (from geometry_msgs)
			ROS message containing the extended Kalman filter output value for the velocity of the drone.
		"""
		self.ekf_vel = enu_to_ned(np.array([[msg.twist.linear.x], [msg.twist.linear.y], [msg.twist.linear.z]]))






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
		self.ekf_att_euler = enu_to_ned(np.asarray(euler_from_quaternion(ros_q)).reshape(3,1))






	def update_angular_velocity(self, msg):
		"""
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to the local NED frame since the ROS message stores the angular velocity of the drone in the local ENU frame.

		Parameters
		----------
		msg : TwistStamped (from geometry_msgs)
			ROS message containing the the extended Kalman filter output value for the angular velocity of the vehicle.
		"""
		self.ekf_ang_vel = enu_to_ned(np.array([[msg.twist.angular.x], [msg.twist.angular.y], [msg.twist.angular.z]]))






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






	def start_window(self):
		"""
		Creates a window to display the time evolution of the selected physical quantities. 

		Defines the methods responsible for plotting the static and dynamic elements of the window.
		Static elements only have to be plotted once. Dynamic elements must be plotted at the frequency selected by the user.
		"""
		self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(16,9))
		self.line1, = self.ax1.plot([], [], color='black')
		self.line2, = self.ax1.plot([], [], color='blue')
		self.line3, = self.ax2.plot([], [], color='black')
		self.line4, = self.ax2.plot([], [], color='blue')
		self.line5, = self.ax3.plot([], [], color='black')
		self.line6, = self.ax3.plot([], [], color='blue')
		
		if self.graph_type == 'pos':
			graph = animation.FuncAnimation(self.fig, self.dynamic_plot, interval=1000/self.update_freq, init_func=self.static_plot_pos, blit=True)
		elif self.graph_type == 'vel':
			graph = animation.FuncAnimation(self.fig, self.dynamic_plot, interval=1000/self.update_freq, init_func=self.static_plot_vel, blit=True)
		elif self.graph_type == 'att':	
			graph = animation.FuncAnimation(self.fig, self.dynamic_plot, interval=1000/self.update_freq, init_func=self.static_plot_att, blit=True)
		elif self.graph_type == 'ang_vel':
			graph = animation.FuncAnimation(self.fig, self.dynamic_plot, interval=1000/self.update_freq, init_func=self.static_plot_ang_vel, blit=True)

		plt.show()






	def static_plot_pos(self):
		"""
		Plots the static elements of the position monitoring window.

		The static elements are the background of the window, the title of the plots, the axis, the grid, and the label of the axis.
		"""
		rect = self.fig.patch
		rect.set_facecolor('lightblue')

		plt.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.92, wspace=None, hspace=None)

		self.line1.set_label('Reference')
		self.line2.set_label('Real')
		self.ax1.legend(loc='upper left', prop={'size': 9})
		self.line3.set_label('Reference')
		self.line4.set_label('Real')
		self.ax2.legend(loc='upper left', prop={'size': 9})
		self.line5.set_label('Reference')
		self.line6.set_label('Real')
		self.ax3.legend(loc='upper left', prop={'size': 9})

		self.ax1.set_facecolor('whitesmoke')
		self.ax1.set_title('Real Time NED Position Plots: '+self.drone_ns, pad=10, fontsize=20, fontweight='bold')
		self.ax1.set_xlim(-self.time_window, 0)
		self.ax1.tick_params(axis='x', which='major', labelsize=8)
		self.ax1.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax1.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax1.set_ylabel('North [m]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax1.set_ylim(-3, 3)
		self.ax1.tick_params(axis='y', which='major', labelsize=8)
		self.ax1.tick_params(axis='y', which='minor', labelsize=4)
		self.ax1.yaxis.set_major_locator(MultipleLocator(1))
		self.ax1.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax1.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax1.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax2.set_facecolor('whitesmoke')
		self.ax2.set_xlim(-self.time_window, 0)
		self.ax2.tick_params(axis='x', which='major', labelsize=8)
		self.ax2.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax2.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax2.set_ylabel('East [m]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax2.set_ylim(-2, 2)
		self.ax2.tick_params(axis='y', which='major', labelsize=8)
		self.ax2.tick_params(axis='y', which='minor', labelsize=4)
		self.ax2.yaxis.set_major_locator(MultipleLocator(1))
		self.ax2.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax2.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax2.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax3.set_facecolor('whitesmoke')
		self.ax3.set_xlabel('Time: Last '+str(self.time_window)+' [seconds]', labelpad=5, fontsize=15, fontweight='bold')
		self.ax3.set_xlim(-self.time_window, 0)
		self.ax3.tick_params(axis='x', which='major', labelsize=8)
		self.ax3.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax3.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax3.set_ylabel('-Down [m]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax3.set_ylim(0, 2.5)
		self.ax3.tick_params(axis='y', which='major', labelsize=8)
		self.ax3.tick_params(axis='y', which='minor', labelsize=4)
		self.ax3.yaxis.set_major_locator(MultipleLocator(1))
		self.ax3.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax3.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax3.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		return self.line1, self.line2, self.line3, self.line4, self.line5, self.line6






	def static_plot_vel(self):
		"""
		Plots the static elements of the velocity monitoring window.

		The static elements are the background of the window, the title of the plots, the axis, the grid, and the label of the axis.
		"""
		rect = self.fig.patch
		rect.set_facecolor('lightblue')

		plt.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.92, wspace=None, hspace=None)

		self.line1.set_label('Reference')
		self.line2.set_label('Real')
		self.ax1.legend(loc='upper left', prop={'size': 9})
		self.line3.set_label('Reference')
		self.line4.set_label('Real')
		self.ax2.legend(loc='upper left', prop={'size': 9})
		self.line5.set_label('Reference')
		self.line6.set_label('Real')
		self.ax3.legend(loc='upper left', prop={'size': 9})

		self.ax1.set_facecolor('whitesmoke')
		self.ax1.set_title('Real Time NED Velocity Plots: '+self.drone_ns, pad=10, fontsize=20, fontweight='bold')
		self.ax1.set_xlim(-self.time_window, 0)
		self.ax1.tick_params(axis='x', which='major', labelsize=8)
		self.ax1.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax1.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax1.set_ylabel('Vel. North [m/s]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax1.set_ylim(-3, 3)
		self.ax1.tick_params(axis='y', which='major', labelsize=8)
		self.ax1.tick_params(axis='y', which='minor', labelsize=4)
		self.ax1.yaxis.set_major_locator(MultipleLocator(1))
		self.ax1.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax1.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax1.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax2.set_facecolor('whitesmoke')
		self.ax2.set_xlim(-self.time_window, 0)
		self.ax2.tick_params(axis='x', which='major', labelsize=8)
		self.ax2.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax2.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax2.set_ylabel('Vel. East [m/s]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax2.set_ylim(-3, 3)
		self.ax2.tick_params(axis='y', which='major', labelsize=8)
		self.ax2.tick_params(axis='y', which='minor', labelsize=4)
		self.ax2.yaxis.set_major_locator(MultipleLocator(1))
		self.ax2.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax2.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax2.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax3.set_facecolor('whitesmoke')
		self.ax3.set_xlabel('Time: Last '+str(self.time_window)+' [seconds]', labelpad=5, fontsize=15, fontweight='bold')
		self.ax3.set_xlim(-self.time_window, 0)
		self.ax3.tick_params(axis='x', which='major', labelsize=8)
		self.ax3.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax3.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax3.set_ylabel('-Vel. Down [m/s]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax3.set_ylim(-3, 3)
		self.ax3.tick_params(axis='y', which='major', labelsize=8)
		self.ax3.tick_params(axis='y', which='minor', labelsize=4)
		self.ax3.yaxis.set_major_locator(MultipleLocator(1))
		self.ax3.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax3.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax3.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		return self.line1, self.line2, self.line3, self.line4, self.line5, self.line6






	def static_plot_att(self):
		"""
		Plots the static elements of the attitude monitoring window.

		The static elements are the background of the window, the title of the plots, the axis, the grid, and the label of the axis.
		"""
		rect = self.fig.patch
		rect.set_facecolor('lightblue')

		plt.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.92, wspace=None, hspace=None)

		self.line1.set_label('Reference')
		self.line2.set_label('Real')
		self.ax1.legend(loc='upper left', prop={'size': 9})
		self.line3.set_label('Reference')
		self.line4.set_label('Real')
		self.ax2.legend(loc='upper left', prop={'size': 9})
		self.line5.set_label('Reference')
		self.line6.set_label('Real')
		self.ax3.legend(loc='upper left', prop={'size': 9})

		self.ax1.set_facecolor('whitesmoke')
		self.ax1.set_title('Real Time Attitude Plots: '+self.drone_ns, pad=10, fontsize=20, fontweight='bold')
		self.ax1.set_xlim(-self.time_window, 0)
		self.ax1.tick_params(axis='x', which='major', labelsize=8)
		self.ax1.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax1.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax1.set_ylabel('Roll [degrees]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax1.set_ylim(-45, 45)
		self.ax1.tick_params(axis='y', which='major', labelsize=8)
		self.ax1.tick_params(axis='y', which='minor', labelsize=4)
		self.ax1.yaxis.set_major_locator(MultipleLocator(15))
		self.ax1.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax1.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax1.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax2.set_facecolor('whitesmoke')
		self.ax2.set_xlim(-self.time_window, 0)
		self.ax2.tick_params(axis='x', which='major', labelsize=8)
		self.ax2.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax2.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax2.set_ylabel('Pitch [degrees]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax2.set_ylim(-45, 45)
		self.ax2.tick_params(axis='y', which='major', labelsize=8)
		self.ax2.tick_params(axis='y', which='minor', labelsize=4)
		self.ax2.yaxis.set_major_locator(MultipleLocator(15))
		self.ax2.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax2.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax2.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax3.set_facecolor('whitesmoke')
		self.ax3.set_xlabel('Time: Last '+str(self.time_window)+' [seconds]', labelpad=5, fontsize=15, fontweight='bold')
		self.ax3.set_xlim(-self.time_window, 0)
		self.ax3.tick_params(axis='x', which='major', labelsize=8)
		self.ax3.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax3.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax3.set_ylabel('Yaw [degrees]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax3.set_ylim(-180, 180)
		self.ax3.tick_params(axis='y', which='major', labelsize=8)
		self.ax3.tick_params(axis='y', which='minor', labelsize=4)
		self.ax3.yaxis.set_major_locator(MultipleLocator(60))
		self.ax3.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax3.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax3.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		return self.line1, self.line2, self.line3, self.line4, self.line5, self.line6






	def static_plot_ang_vel(self):
		"""
		Plots the static elements of the angular velocity monitoring window.

		The static elements are the background of the window, the title of the plots, the axis, the grid, and the label of the axis.
		"""
		rect = self.fig.patch
		rect.set_facecolor('lightblue')

		plt.subplots_adjust(left=0.08, bottom=0.08, right=0.95, top=0.92, wspace=None, hspace=None)

		self.line1.set_label('Reference')
		self.line2.set_label('Real')
		self.ax1.legend(loc='upper left', prop={'size': 9})
		self.line3.set_label('Reference')
		self.line4.set_label('Real')
		self.ax2.legend(loc='upper left', prop={'size': 9})
		self.line5.set_label('Reference')
		self.line6.set_label('Real')
		self.ax3.legend(loc='upper left', prop={'size': 9})

		self.ax1.set_facecolor('whitesmoke')
		self.ax1.set_title('Real Time Angular Velocity Plots '+self.drone_ns, pad=10, fontsize=20, fontweight='bold')
		self.ax1.set_xlim(-self.time_window, 0)
		self.ax1.tick_params(axis='x', which='major', labelsize=8)
		self.ax1.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax1.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax1.set_ylabel('Roll Rate [deg/s]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax1.set_ylim(-180, 180)
		self.ax1.tick_params(axis='y', which='major', labelsize=8)
		self.ax1.tick_params(axis='y', which='minor', labelsize=4)
		self.ax1.yaxis.set_major_locator(MultipleLocator(60))
		self.ax1.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax1.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax1.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax2.set_facecolor('whitesmoke')
		self.ax2.set_xlim(-self.time_window, 0)
		self.ax2.tick_params(axis='x', which='major', labelsize=8)
		self.ax2.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax2.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax2.set_ylabel('Pitch Rate [deg/s]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax2.set_ylim(-180, 180)
		self.ax2.tick_params(axis='y', which='major', labelsize=8)
		self.ax2.tick_params(axis='y', which='minor', labelsize=4)
		self.ax2.yaxis.set_major_locator(MultipleLocator(60))
		self.ax2.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax2.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax2.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		self.ax3.set_facecolor('whitesmoke')
		self.ax3.set_xlabel('Time: Last '+str(self.time_window)+' [seconds]', labelpad=5, fontsize=15, fontweight='bold')
		self.ax3.set_xlim(-self.time_window, 0)
		self.ax3.tick_params(axis='x', which='major', labelsize=8)
		self.ax3.xaxis.set_major_locator(MultipleLocator(self.time_window/5))
		self.ax3.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax3.set_ylabel('Yaw Rate [deg/s]', labelpad=8, fontsize=14, fontweight='bold')
		self.ax3.set_ylim(-180, 180)
		self.ax3.tick_params(axis='y', which='major', labelsize=8)
		self.ax3.tick_params(axis='y', which='minor', labelsize=4)
		self.ax3.yaxis.set_major_locator(MultipleLocator(60))
		self.ax3.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax3.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax3.grid(axis='y', which='minor', color='#000000', linestyle=':')
		
		return self.line1, self.line2, self.line3, self.line4, self.line5, self.line6






	def dynamic_plot(self, _):
		"""
		Plots the dynamic elements of the monitoring window.

		The dynamic elements are the plot lines that represent the time evolution of the desired and measured control variables.
		"""
		if self.graph_type == 'pos':
			self.y2 = self.y2[1:]; self.y2.append(self.ekf_pos[0][0])
			self.y4 = self.y4[1:]; self.y4.append(self.ekf_pos[1][0])
			self.y6 = self.y6[1:]; self.y6.append(-self.ekf_pos[2][0])
			self.line2.set_data(self.x, self.y2)
			self.line4.set_data(self.x, self.y4)
			self.line6.set_data(self.x, self.y6)

			if self.pos_ref_flag != False:
				self.y1 = self.y1[1:]; self.y1.append(self.pos_ref[0][0])
				self.y3 = self.y3[1:]; self.y3.append(self.pos_ref[1][0])
				self.y5 = self.y5[1:]; self.y5.append(-self.pos_ref[2][0])
				self.line1.set_data(self.x, self.y1)
				self.line3.set_data(self.x, self.y3)
				self.line5.set_data(self.x, self.y5)

		elif self.graph_type == 'vel':
			self.y2 = self.y2[1:]; self.y2.append(self.ekf_vel[0][0])
			self.y4 = self.y4[1:]; self.y4.append(self.ekf_vel[1][0])
			self.y6 = self.y6[1:]; self.y6.append(-self.ekf_vel[2][0])
			self.line2.set_data(self.x, self.y2)
			self.line4.set_data(self.x, self.y4)
			self.line6.set_data(self.x, self.y6)

			if self.vel_ref_flag != False:
				self.y1 = self.y1[1:]; self.y1.append(self.vel_ref[0][0])
				self.y3 = self.y3[1:]; self.y3.append(self.vel_ref[1][0])
				self.y5 = self.y5[1:]; self.y5.append(-self.vel_ref[2][0])
				self.line1.set_data(self.x, self.y1)
				self.line3.set_data(self.x, self.y3)
				self.line5.set_data(self.x, self.y5)

		elif self.graph_type == 'att':
			self.y2 = self.y2[1:]; self.y2.append(math.degrees(self.ekf_att_euler[0][0]))
			self.y4 = self.y4[1:]; self.y4.append(math.degrees(self.ekf_att_euler[1][0]))
			self.y6 = self.y6[1:]; self.y6.append(math.degrees(self.ekf_att_euler[2][0]))
			self.line2.set_data(self.x, self.y2)
			self.line4.set_data(self.x, self.y4)
			self.line6.set_data(self.x, self.y6)

			if self.att_euler_ref_flag != False:
				self.y1 = self.y1[1:]; self.y1.append(math.degrees(self.att_euler_ref[0][0]))
				self.y3 = self.y3[1:]; self.y3.append(math.degrees(self.att_euler_ref[1][0]))
				self.y5 = self.y5[1:]; self.y5.append(math.degrees(self.att_euler_ref[2][0]))
				self.line1.set_data(self.x, self.y1)
				self.line3.set_data(self.x, self.y3)
				self.line5.set_data(self.x, self.y5)

		elif self.graph_type == 'ang_vel':
			self.y2 = self.y2[1:]; self.y2.append(math.degrees(self.ekf_ang_vel[0][0]))
			self.y4 = self.y4[1:]; self.y4.append(math.degrees(self.ekf_ang_vel[1][0]))
			self.y6 = self.y6[1:]; self.y6.append(math.degrees(self.ekf_ang_vel[2][0]))
			self.line2.set_data(self.x, self.y2)
			self.line4.set_data(self.x, self.y4)
			self.line6.set_data(self.x, self.y6)

			if self.ang_vel_ref_flag != False:
				self.y1 = self.y1[1:]; self.y1.append(math.degrees(self.ang_vel_ref[0][0]))
				self.y3 = self.y3[1:]; self.y3.append(math.degrees(self.ang_vel_ref[1][0]))
				self.y5 = self.y5[1:]; self.y5.append(math.degrees(self.ang_vel_ref[2][0]))
				self.line1.set_data(self.x, self.y1)
				self.line3.set_data(self.x, self.y3)
				self.line5.set_data(self.x, self.y5)
		
		return self.line1, self.line2, self.line3, self.line4, self.line5, self.line6






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
	Retrieves, from the yaml configuration file associated with this tool, the update frequency, the limits of the time axis, and the sources of the control references.

	Returns
	-------
	update_freq
		Update frequency of the generated window.
	source_pos_ref
		States the source of the position control references. 
	source_vel_ref
		States the source of the velocity control references. 
	source_att_ref
		States the source of the attitude control references. 
	source_ang_vel_ref
		States the source of the angular velocity control references. 
	time_window
		Limits of the time axis.
	"""
	with open(os.path.expanduser('~/catkin_ws_python/src/tools/src/real_time_monitor.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		update_freq = configs_dict['update_frequency']
		source_pos_ref = configs_dict['source_position_ref']
		source_vel_ref = configs_dict['source_velocity_ref']
		source_att_ref = configs_dict['source_attitude_ref']
		source_ang_vel_ref = configs_dict['source_angular_velocity_ref']
		time_window = configs_dict['x_axis_time_window']

	return update_freq, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref, time_window






if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Reads the yaml configuration file associated with this tool. 
	Creates a window that displays the time evolution of the selected physical quantities.
	"""
	rospy.init_node('real_time_monitor', anonymous=True)

	update_freq, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref, time_window = read_yaml_configuration_file()
	monitor = REAL_TIME_MONITOR(sys.argv[1], sys.argv[2], update_freq, source_pos_ref, source_vel_ref, source_att_ref, source_ang_vel_ref, time_window)

#!/usr/bin/env python3
"""
Lightweight software program that reproduces, in 3D, the evolution of the position of the drones during the experiment.

The visualization tools are crucial for the designed testing framework because, by reproducing the 3D motion of the vehicles during and after the experiments, they enable the user to run simulations without the graphical user interface of Gazebo and still monitor the behavior of the vehicles. This results in computationally lighter simulations. Additionally, these tools also enable the user to visualize, in a single window, the 3D motion of all the drones of the experience, regardless of whether they are real or being simulated in any of the computers of the local network.
"""




#Import python libraries.
import os
import sys
import math
import yaml
import rospy
import threading
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.animation as animation
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)




# Import ROS messages.
from geometry_msgs.msg import PoseStamped




class VISUALIZATION_TOOL():
	"""
	Class responsible for displaying, in a window, the time evolution of the 3D position of the drones of the experiment.

	Methods
	-------
	init_dicts_pos(self)
		Starts the dictionaries that store the last tail_extension+1 positions of each one of the drones.
	init_position_updates(self)
		Subscribes to the ROS topics with the position of each one of the drones.
	update_position(self, msg, ns)
		Updates the dictionaries that store the last positions of the drone identified by the ns argument.
	start_visualization_window(self)
		Creates a window to display the time evolution of the 3D position of the drones of the experiment.
	static_plot_part(self)
		Plots the static elements of the window that displays the time evolution of the 3D position of the drones of the experiment.
	dynamic_plot_part(self, _)
		Plots the dynamic elements of the window that displays the time evolution of the 3D position of the drones of the experiment.
	"""
	def __init__(self, tail_extension, azimuth, elevation, colors):
		"""
		Constructor of the REAL_TIME_MONITOR class. 

		Starts a thread that subscribes to the ROS topics with the position of each one of the drones.
		Calls the method that creates and updates the visualization window.

		Parameters
		----------
		tail_extension : int
			States the number of last positions of the trajetory of the drone to be plotted as a black line.
		azimuth : float
			States the azimuth of the desired 3D view.
		elevation : float
			States the elevation of the desired 3D view.
		colors : list
			States the desired colors for the drones.
		"""
		self.freq = 30
		self.ax = None		
		self.fig = None
		self.azim = azimuth
		self.elev = elevation
		self.colors = colors
		self.lines = []
		self.tail_lines = []
		self.dict_pos_north = {}
		self.dict_pos_east = {}
		self.dict_pos_down = {}
		self.tail_extension = tail_extension
		
		self.init_dicts_pos()
		threading.Thread(target=self.init_position_updates, daemon=True).start()
		rate = rospy.Rate(1); rate.sleep()

		self.start_visualization_window()





	def init_dicts_pos(self):
		"""
		Starts the dictionaries that store the last tail_extension+1 positions of each one of the drones.
		"""
		for ns in sys.argv[1:]:
			self.dict_pos_north[ns] = [0]*(tail_extension+1)
			self.dict_pos_east[ns] = [0]*(tail_extension+1)
			self.dict_pos_down[ns] = [0]*(tail_extension+1)





	def init_position_updates(self):
		"""
		Subscribes to the ROS topics with the position of each one of the drones.
		"""
		for ns in sys.argv[1:]:
			pos_sub = rospy.Subscriber(ns+'/mavros/local_position/pose', PoseStamped, self.update_position, ns)





	def update_position(self, msg, ns):
		"""
		Updates the dictionaries that store the last positions of the drone with the namespace passed in the ns argument.

		Parameters
		----------
		msg : PoseStamped (from geometry_msgs) 
			ROS message containing the extended Kalman filter output value for the position of the drone.
		ns : str
			ROS namespace of the vehicle.
		"""
		pos = enu_to_ned(np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]]))
	
		aux = self.dict_pos_north[ns][1:]
		aux.append(pos[0][0])
		self.dict_pos_north[ns] = aux

		aux = self.dict_pos_east[ns][1:]
		aux.append(pos[1][0])
		self.dict_pos_east[ns] = aux

		aux = self.dict_pos_down[ns][1:]
		aux.append(-pos[2][0])
		self.dict_pos_down[ns] = aux





	def start_visualization_window(self):
		"""
		Creates a window to display the time evolution of the 3D position of the drones of the experiment.

		Defines the methods responsible for plotting the static and dynamic elements of the window.
		Static elements only have to be plotted once. Dynamic elements must be plotted at the frequency selected in the constructor of the object of this class.
		"""
		self.fig = plt.figure(figsize=(16,9))
		self.ax = self.fig.add_subplot(111, projection='3d')
		colors=['blue','red']
		i=0
		for ns in sys.argv[1:]:
			line, = self.ax.plot([], [], [], 'o', markersize=14, markerfacecolor=self.colors[i], markeredgewidth=1.5, markeredgecolor='black'); 
			self.lines.append(line)

			tail_line, = self.ax.plot([],[],[], color='black')
			self.tail_lines.append(tail_line)		
			i+=1

		graph=animation.FuncAnimation(self.fig, self.dynamic_plot_part, interval=1000/self.freq, init_func=self.static_plot_part, blit=True)
		plt.show()





	def static_plot_part(self):
		"""
		Plots the static elements of the window that displays the time evolution of the 3D position of the drones of the experiment.

		The static elements are the background of the window, the title of the plot, the axis, the grid, and the label of the axis.
		"""
		rect = self.fig.patch
		rect.set_facecolor('lightsteelblue')

		self.ax.view_init(elev=self.elev, azim=self.azim)
	
		self.ax.set_facecolor('whitesmoke')
		self.ax.set_title('Position of the Drones', pad=100, fontsize=20, fontweight='bold')

		self.ax.set_xlabel("Position East [m]", labelpad=15, fontsize=15, fontweight='bold')
		self.ax.set_xlim(-2.5, 2.5)
		self.ax.tick_params(axis='x', which='major', labelsize=8)
		self.ax.tick_params(axis='y', which='minor', labelsize=4)
		self.ax.xaxis.set_major_locator(MultipleLocator(1))
		self.ax.xaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax.grid(axis='x', which='major', color='#000000', linestyle='--')
		self.ax.grid(axis='x', which='minor', color='#000000', linestyle=':')

		self.ax.set_ylabel("Position North [m]", labelpad=15, fontsize=15, fontweight='bold')
		self.ax.set_ylim(-3.5, 3.5)
		self.ax.tick_params(axis='y', which='major', labelsize=8)
		self.ax.tick_params(axis='y', which='minor', labelsize=4)
		self.ax.yaxis.set_major_locator(MultipleLocator(1))
		self.ax.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax.grid(axis='y', which='major', color='#000000', linestyle='--')
		self.ax.grid(axis='y', which='minor', color='#000000', linestyle=':')

		self.ax.set_zlabel("-Position Down [m]", labelpad=15, fontsize=15, fontweight='bold')
		self.ax.set_zlim(0, 2.8)
		self.ax.tick_params(axis='z', which='major', labelsize=8)
		self.ax.tick_params(axis='z', which='minor', labelsize=4)
		self.ax.yaxis.set_major_locator(MultipleLocator(1))
		self.ax.yaxis.set_minor_locator(AutoMinorLocator(2))
		self.ax.grid(axis='z', which='major', color='#000000', linestyle='--')
		self.ax.grid(axis='z', which='minor', color='#000000', linestyle=':')
				
		return (*self.lines, *self.tail_lines)





	def dynamic_plot_part(self, _):
		"""
		Plots the dynamic elements of the window that displays the time evolution of the 3D position of the drones of the experiment.

		The dynamic elements are the spheres that represent the drones and the tail lines that represent the last positions of the trajetory performed the drone.
		"""
		i=0
		for ns in sys.argv[1:]:
			x = self.dict_pos_east[ns][-1]
			y = self.dict_pos_north[ns][-1]
			z = self.dict_pos_down[ns][-1]
			tx = self.dict_pos_east[ns][:-1]
			ty = self.dict_pos_north[ns][:-1]
			tz = self.dict_pos_down[ns][:-1]

			self.lines[i].set_data(x, y)
			self.tail_lines[i].set_data(tx, ty)

			self.lines[i].set_3d_properties(z)
			self.tail_lines[i].set_3d_properties(tz)

			i+=1
	
		return (*self.lines, *self.tail_lines)





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
	Retrieves, from the yaml configuration file associated with this tool, the properties desired by the user for the 3D representation of the drones.

	Returns
	-------
	tail_extension
		States the number of last positions of the trajetory of the drone plotted as a black line.
	azimuth
		States the azimuth of the desired 3D view.
	elavation
		States the elevation of the desired 3D view.
	colors
		States the desired colors for the drones.
	"""
	with open(os.path.expanduser('~/catkin_ws_python/src/tools/src/visualization_tool.yaml')) as config_file:
		configs_dict = yaml.load(config_file, Loader=yaml.FullLoader)
		tail_extension = configs_dict['tail_extension']
		azimuth = configs_dict['azimuth']
		elevation = configs_dict['elevation']
		colors = []
		for i in range(1, len(sys.argv)):
			colors.append(configs_dict['color_'+str(i)])

	return tail_extension, azimuth, elevation, colors





if __name__ == "__main__":
	"""
	Main function.

	Starts a ROS node. Reads the yaml configuration file associated with this tool. 
	Starts an object of the VISUALIZATION_TOOL class that creates a window that displays the time evolution of the 3D position of the drones of the experiment.
	"""
	rospy.init_node('python_visualization_tool', anonymous=True)
	
	tail_extension, azimuth, elevation, colors = read_yaml_configuration_file()

	VISUALIZATION_TOOL(tail_extension, azimuth, elevation, colors)

#!/usr/bin/env python3





# Import the methods from the other files.
from offboard import OFFBOARD
from telemetry import TELEMETRY





# Import python libraries.
import asyncio
import threading
from mavsdk import System





class UAV (TELEMETRY, OFFBOARD):
	"""
	Class used to represent an UAV.

	Attributes
	----------
	ekf : object of the EKF class
		Stores the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	act : object of the ACTUATORS class
		Stores the current values applied to the mixers and/or actuators (motors and control devices) of the vehicle.
	info : object of the DRONE_INFO class
		Stores physical properties and the flight status of the vehicle.
	"""
	def __init__(self, drone_udp, mass, radius, height, num_rotors, thrust_curve):
		"""
		Constructor of the UAV class.
		
		Parameters
		----------
		drone_udp : str
			UDP address of the PX4 autopilot.
		mass : str
			Mass of the drone.
		radius : str
			Radius of the vehicle.
		height : str
			Height of the drone.
		num_rotors : str
			Number of rotors of the drone.
		thrust_curve : str
			Thrust curve of the vehicle.
		"""
		self.ekf = EKF()
		self.act = ACTUATORS()
		self.info = DRONE_INFO()

		self.info.drone_udp = "udp://:"+drone_udp
		self.info.mass = float(mass)
		self.info.radius = float(radius)
		self.info.height = float(height)
		self.info.num_rotors = int(num_rotors)
		self.info.thrust_curve = thrust_curve
		self.system = System(port=50050+int(drone_udp[-3:]))





class EKF():
	"""
	Class used to store the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot. All variables are in SI units.

	Attributes
	----------
	pos : np.array of floats with shape (3,1)
		Position of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	vel : np.array of floats with shape (3,1)
		Linear velocity of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	vel_body : np.array of floats with shape (3,1)
		Linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	att_q : np.array of floats with shape (4,1)
		Attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.
	att_euler : np.array of floats with shape (3,1)
		Attitude of the drone, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.
	ang_vel : np.array of floats with shape (3,1)
		Angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	"""
	def __init__(self):
		"""
		Constructor of the EKF class.
		"""
		self.pos = None
		self.vel = None
		self.vel_body = None
		self.att_q = None
		self.att_euler = None
		self.ang_vel = None





class ACTUATORS():
	"""
	Class used to store the current values applied to the mixer and/or actuators (motors and control devices) of the drone.
	More information available at https://dev.px4.io/v1.9.0/en/concept/mixing.html.

	Attributes
	----------
	group : int
		States the group of the active motors and servos of the drone.
	output : list
		Stores the normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
	"""
	def __init__(self):
		"""
		Constructor of the ACTUATORS class.
		"""
		self.group = None
		self.output = None





class DRONE_INFO():
	"""
	Class used to store physical properties and the flight status of the drone. All variables are in SI units.

	Attributes
	----------
	drone_udp : str
		UDP address of the PX4 autopilot.
	mass : float
		Mass of the drone.
	radius : float
		Radius of the drone.
	height : float
		Height of the drone.
	num_rotors : int
		Number of rotors of the drone.
	thrust_curve : str
		Thrust curve of the drone.
	flight_mode : str
		Current flight mode of the drone. The list of flight modes is available at http://wiki.ros.org/mavros/CustomModes.
	is_armed : bool
		Stores the armed state of the vehicle. If True, the drone is armed.
	is_landed : bool
		Stores the landed state of the vehicle. If True, the drone is landed.
	battery : float
		Remaining battery percentage.
	"""
	def __init__(self):
		"""
		Constructor of the DRONE_INFO class.
		"""
		self.drone_udp = None
		self.mass = None
		self.radius = None
		self.height = None
		self.num_rotors = None
		self.thrust_curve = None
		self.flight_mode = None
		self.is_connected = False
		self.is_armed = None
		self.is_landed = None
		self.battery = None

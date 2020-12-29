#!/usr/bin/env python3





# Import the methods from the other files.
from offboard import OFFBOARD
from telemetry import TELEMETRY





# Import python libraries.
import rospy
import threading





class UAV (TELEMETRY, OFFBOARD):
	"""
	Class used to represent an UAV.

	Attributes
	----------
	ekf : object of the EKF class
		Stores the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	sen : object of the SENSORS class
		Stores the raw sensor measurements and the MOCAP pose of the drone.
	act : object of the ACTUATORS class
		Stores the current values applied to the mixers and/or actuators (motors and control devices) of the vehicle.
	info : object of the DRONE_INFO class
		Stores physical properties and the flight status of the vehicle.
	"""
	def __init__(self, drone_ns, mass, radius, height, num_rotors, thrust_curve):
		"""
		Constructor of the UAV class. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
		Awaits until the connection with the drone is established.
		
		Parameters
		----------
		drone_ns : str
			ROS namespace where the data from the PX4 autopilot and the MOCAP system is encapsulated.
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
		self.sen = SENSORS()
		self.act = ACTUATORS()
		self.info = DRONE_INFO()
		
		self.info.drone_ns = drone_ns
		self.info.mass = float(mass)
		self.info.radius = float(radius)
		self.info.height = float(height)
		self.info.num_rotors = int(num_rotors)
		self.info.thrust_curve = thrust_curve
		
		threading.Thread(target=self.init_telemetry, daemon=True).start()

		print("\nConnecting to the drone...")
		while self.info.is_connected==False:
			rate=rospy.Rate(2); rate.sleep()
		print("Connection established!")
		rate=rospy.Rate(1/5); rate.sleep()





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





class SENSORS():
	"""
	Class used to store the raw sensors measurements and the MOCAP pose of the drone.

	Attributes
	----------
	imu : object of the IMU class
		Stores the raw measurements provided by the IMU.
	mocap : object of the MOCAP class
		Stores the position and attitude of the drone provided by the motion capture system.
	gps : object of the GPS class
		Stores the raw measurements provided by the GPS sensor.
	baro : object of the BAROMETER class
		Stores the raw measurements provided by the barometer.
	emu : object of the EMULATOR class
		Stores the raw measurements provided by emulated sensors.
	"""
	def __init__(self):
		"""
		Constructor of the SENSORS class.
		"""
		self.imu = IMU()
		self.mocap = MOCAP()
		self.gps = GPS()
		self.baro = BAROMETER()
		self.emu = EMULATED()





class IMU():
	"""
	Class used to store the raw measurements provided by the IMU. All variables are in SI units.

	Attributes
	----------
	acc_body : np.array of floats with shape (3,1)
		Linear acceleration of the vehicle, in body NED coordinates, measured by the IMU.
	ang_vel : np.array of floats with shape (3,1)
		Angular velocity of the drone measured by the IMU.
	mag : np.array of floats with shape (3,1)
		Magnetic field vector, in body NED coordinates, measured by the IMU. Expressed in Teslas.
	"""
	def __init__(self):
		"""
		Constructor of the IMU class.
		"""
		self.acc_body = None
		self.ang_vel = None
		self.mag = None





class MOCAP():
	"""
	Class used to store the position and attitude of the drone provided by the motion capture system. All variables are in SI units.

	Attributes
	----------
	pos : np.array of floats with shape (3,1)
		Position of the vehicle, in local NED coordinates, provided by the motion capture system.
	att_q : np.array of floats with shape (4,1)
		Attitude of the vehicle, expressed in quaternions, provided by the motion capture system.
	att_euler : np.array of floats with shape (3,1)
		Attitude of the drone, expressed in Euler angles, provided by the motion capture system.
	"""
	def __init__(self):
		"""
		Constructor of the MOCAP class.
		"""
		self.pos = None
		self.att_q = None
		self.att_euler = None





class GPS():
	"""
	Class used to store the raw measurements provided by the GPS sensor. All variables are in SI units.

	Attributes
	----------
	pos : np.array of floats with shape (3,1)	
		Position of the vehicle, in gps coordinates, provided by the GPS sensor.
	"""
	def __init__(self):
		"""
		Constructor of the GPS class.
		"""
		self.pos = None





class BAROMETER():
	"""
	Class used to store the raw measurements provided by the barometer. All variables are in SI units.

	Attributes
	----------
	pressure : float
		Static pressure measured by the barometer.
	temperature : float
		Temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.
	alt : float
		Altitude of the vehicle, above mean sea level, computed through the barometric atmospheric pressure and the temperature.
	"""
	def __init__(self):
		"""
		Constructor of the BAROMETER class.
		"""
		self.pressure = None
		self.temperature = None
		self.alt = None





class EMULATED():
	"""
	Class used to store the raw measurements provided by emulated sensors.
	
	Attributes
	----------
	rel_pos : np.array of floats with shape (n,3,1)
		Relative position of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative position sensor.
	rel_vel : np.array of floats with shape (n,3,1)
		Relative velocity of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative velocity sensor.
	"""
	def __init__(self):
		"""
		Constructor of the EMULATED class.
		"""
		self.rel_pos = None
		self.rel_vel = None





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
	drone_ns : str
		ROS namespace where the data from the PX4 autopilot and the MOCAP system is encapsulated.
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
	is_connected : bool
		States if the system is connected to the PX4 autopilot. 
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
		self.drone_ns = None
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

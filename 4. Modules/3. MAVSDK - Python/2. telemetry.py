#!/usr/bin/env python3





# Import python libraries
import math
import asyncio
import numpy as np





class TELEMETRY:
	"""
	Stores the methods responsible for keeping the variables of the UAV class up to date.

	Methods
	-------
	init_telemetry(self)
		Defines the callback methods responsible for updating each variable of the UAV class.
	update_position_and_velocity(self)
		Updates the variables that store the position and linear velocity of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity_body(self)
		Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude_quaternion(self)
		Updates the variable that stores the attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude_euler(self)
		Updates the variable that stores the attitude of the vehicle, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.
	update_angular_velocity(self)
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_actuator(self)
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
	update_flight_mode(self)
		Updates the variable that stores the current flight mode of the PX4 autopilot of the vehicle. 
	update_status(self)
		Updates the variable that stores the armed state of the vehicle. 
	update_landed(self)
		Updates the variable that stores the landed state of the drone.
	update_battery(self)
		Updates the variable that stores the remaining battery percentage. 
	"""
	async def init_telemetry(self):
		"""
		Defines the callback methods responsible for updating each variable of the UAV class.
		"""
		asyncio.ensure_future(self.update_position_and_velocity())
		asyncio.ensure_future(self.update_velocity_body())
		asyncio.ensure_future(self.update_attitude_quaternion())
		asyncio.ensure_future(self.update_attitude_euler())
		asyncio.ensure_future(self.update_angular_velocity())
		asyncio.ensure_future(self.update_actuator())
		asyncio.ensure_future(self.update_flight_mode())
		asyncio.ensure_future(self.update_status())
		asyncio.ensure_future(self.update_landed())
		asyncio.ensure_future(self.update_battery())





	async def update_position_and_velocity(self):
		"""
		Updates the variables that store the position and linear velocity of the drone provided by the extended Kalman filter of the PX4 autopilot.
		"""
		async for msg in self.system.telemetry.position_velocity_ned():
			self.ekf.pos = np.array([[msg.position.north_m], [msg.position.east_m], [msg.position.down_m]])
			self.ekf.vel = np.array([[msg.velocity.north_m_s], [msg.velocity.east_m_s], [msg.velocity.down_m_s]])





	async def update_velocity_body(self):
		"""
		Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
		"""
		async for msg in self.system.telemetry.odometry():
			self.ekf.vel_body = np.array([[msg.velocity_body.x_m_s], [msg.velocity_body.y_m_s], [msg.velocity_body.z_m_s]])





	async def update_attitude_quaternion(self):
		"""
		Updates the variable that stores the attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.
		"""
		async for msg in self.system.telemetry.attitude_quaternion():
			self.ekf.att_q = np.array([[msg.w], [msg.x], [msg.y], [msg.z]])





	async def update_attitude_euler(self):
		"""
		Updates the variable that stores the attitude of the vehicle, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.

		Makes appropriate convertions to SI units since the MAVSDK library stores angles in degrees.
		"""
		async for msg in self.system.telemetry.attitude_euler():
			self.ekf.att_euler = np.array([[math.radians(msg.roll_deg)], [math.radians(msg.pitch_deg)], [math.radians(msg.yaw_deg)]])





	async def update_angular_velocity(self):
		"""
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
		"""
		async for msg in self.system.telemetry.attitude_angular_velocity_body():
			self.ekf.ang_vel = np.array([[msg.roll_rad_s], [msg.pitch_rad_s], [msg.yaw_rad_s]])





	async def update_actuator(self):
		"""
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
		"""
		async for msg in self.system.telemetry.actuator_control_target():
			self.act.group = msg.group
			self.act.output = msg.controls





	async def update_flight_mode(self):
		"""
		Updates the variable that stores the current flight mode of the PX4 autopilot of the vehicle. 
		"""
		async for msg in self.system.telemetry.flight_mode():
			self.info.flight_mode = msg





	async def update_status(self):
		"""
		Updates the variable that stores the armed state of the vehicle. 
		"""
		async for msg in self.system.telemetry.armed():
			self.info.is_armed = msg





	async def update_landed(self):
		"""
		Updates the variable that stores the landed state of the drone.
		"""
		async for msg in self.system.telemetry.in_air():
			self.info.is_landed = not msg





	async def update_battery(self):
		"""
		Updates the variable that stores the remaining battery percentage. 
		"""
		async for msg in self.system.telemetry.battery():
			self.info.battery = msg.remaining_percent*100

#!/usr/bin/env python3





# Import python libraries
import math
import asyncio
import numpy as np
from scipy.linalg import norm
from mavsdk.offboard import Attitude
from mavsdk.offboard import AttitudeRate
from mavsdk.offboard import PositionNedYaw
from mavsdk.offboard import VelocityNedYaw
from mavsdk.offboard import ActuatorControl
from mavsdk.offboard import VelocityBodyYawspeed





class OFFBOARD:
	"""
	Stores the methods that send offboard commands and offboard control references to the PX4 autopilot of the vehicle.

	Methods
	-------
	connect(self)
		Awaits until the connection with the drone is established. Starts a background thread responsible for keeping all the variables of the UAV class up to date.
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
	set_vel_body_yaw_rate(self, vel_body, yaw_rate, freq)
		Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.
	set_att_thrust(self, att, att_type, thrust, freq)
		Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.
	set_ang_vel_thrust(self, ang_vel, thrust, freq)
		Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.
	set_act(self, group, output, freq)
		Offboard method that sets the values of the mixers and/or actuators of the vehicle.
	disarm_drone(self)
		Disarms the vehicle, if it is not already disarmed.
	auto_land(self)
		Lands the drone, changing its flight mode to auto-land.
	"""
	async def connect(self):
		"""
		Awaits until the connection with the drone is established. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
		"""
		print("\nConnecting to the drone...")
		await self.system.connect(self.info.drone_udp, name='_'+self.info.drone_udp[-3:])
	
		async for msg in self.system.core.connection_state():
			if not msg.is_connected:
				await asyncio.sleep(0.5) 
			else:
				break

		self.info.is_connected = True
		print("Connection established!")

		asyncio.ensure_future(self.init_telemetry())
		await asyncio.sleep(5)





	async def arm_drone(self):
		"""
		Arms the drone, if it is not already armed.
		"""
		if self.info.is_armed == False:
			print('\nArming drone...')
			await self.system.action.arm()
			await asyncio.sleep(1)
			print('Drone armed!') if self.info.is_armed == True else print('Unable to arm the drone.')
		else:
			print('\nThe drone is already armed!')





	async def start_offboard_mode(self):
		"""
		Changes the flight mode of the PX4 autopilot of the drone to offboard.
		"""
		if self.info.flight_mode != 'OFFBOARD':
			await self.set_pos_yaw(self.ekf.pos, self.ekf.att_euler[2][0], 1)
			print('\nChanging to offboard mode...')
			await self.system.offboard.start()
			await asyncio.sleep(1)
			print('Offboard mode activated!') if self.info.flight_mode == 'OFFBOARD' else print('Unable to change to offboard mode.')
		else:
			print('\nThe offboard mode is already set!')





	async def start_offboard_mission(self):
		"""
		Makes the vehicle ready for an offboard experiment by arming it and by changing the flight mode of its PX4 autopilot to offboard.
		"""		
		await self.start_offboard_mode()
		await self.arm_drone()





	async def set_pos_yaw(self, pos, yaw, time):
		"""
		Offboard method that sends position and yaw references to the PX4 autopilot of the the drone.

		Parameters
		----------
		pos : np.array of floats with shape (3,1)
			Desired position for the drone, in local NED coordinates.
		yaw : float
			Desired yaw for the vehicle, in radians.
		time: float
			Time, in seconds, during which the selected position and yaw references will be sent to the PX4 autopilot.
		"""
		t = 0
		while(t < time):
			msg = PositionNedYaw(pos[0][0], pos[1][0], pos[2][0], math.degrees(yaw))
			await self.system.offboard.set_position_ned(msg)
			await asyncio.sleep(1/50)
			t += 1/50





	async def set_vel_yaw(self, vel, yaw, freq):
		"""
		Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.

		Parameters
		----------
		vel : np.array of floats with shape (3,1)
			Desired linear velocity for drone, in local NED coordinates.
		yaw : float
			Desired yaw for the vehicle, in radians.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		msg = VelocityNedYaw(vel[0][0], vel[1][0], vel[2][0], math.degrees(yaw))
		await self.system.offboard.set_velocity_ned(msg)
		await asyncio.sleep(1/freq)





	async def set_vel_body_yaw_rate(self, vel_body, yaw_rate, freq):
		"""
		Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.

		Parameters
		----------
		vel_body : np.array of floats with shape (3,1)
			Desired linear velocity for the drone, in body NED coordinates.
		yaw_rate : float
			Desired yaw rate for the vehicle, in radians per second.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		msg = VelocityBodyYawspeed(vel_body[0][0], vel_body[1][0], vel_body[2][0], math.degrees(yaw_rate))
		await self.system.offboard.set_velocity_body(msg)
		await asyncio.sleep(1/freq)





	async def set_att_thrust(self, att, att_type, thrust, freq):
		"""
		Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.

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
		msg = Attitude(math.degrees(att[0][0]), math.degrees(att[1][0]), math.degrees(att[2][0]), normalize_thrust(thrust, self.info.thrust_curve, norm(self.ekf.vel)))
		await self.system.offboard.set_attitude(msg)
		await asyncio.sleep(1/freq)





	async def set_ang_vel_thrust(self, ang_vel, thrust, freq):
		"""
		Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.

		Converts the thrust from newtons to a normalized value between 0 and 1 through mathematical expression of the thrust curve of the vehicle.

		Parameters
		----------
		ang_vel : np.array of floats with shape (3,1)
			Desired angular velocity for the drone.
		thrust : float
			Desired thrust value in newtons.
		freq : float
			Topic publishing frequency, in Hz.
		"""
		msg = AttitudeRate(math.degrees(ang_vel[0][0]), math.degrees(ang_vel[1][0]), math.degrees(ang_vel[2][0]), normalize_thrust(thrust, self.info.thrust_curve, norm(self.ekf.vel)))
		await self.system.offboard.set_attitude_rate(msg)
		await asyncio.sleep(1/freq)





	async def set_act(self, group, output, freq):
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
		msg = ActuatorControl(output)
		await self.system.offboard.set_actuator_control(msg)
		await asyncio.sleep(1/freq)





	async def disarm_drone(self):
		"""
		Disarms the vehicle, if it is not already disarmed.
		"""
		if self.info.is_armed == True:
			print('\nDisarming drone...')
			await self.system.action.disarm()
			await asyncio.sleep(1)
			print('Drone disarmed!') if self.info.is_armed == False else print('Unable to disarm the drone.')
		else:
			print('\nThe drone is already disarmed!')





	async def auto_land(self):
		"""
		Lands the drone, changing its flight mode to auto-land.
		"""
		if self.info.flight_mode != 'LAND':
			print('\nChanging to auto-land mode...')
			await self.system.action.land()
			await asyncio.sleep(1)
			print('Auto-land mode activated!') if self.info.flight_mode == 'LAND' else print('Unable to change to auto-land mode.')
		else:
			print('\nThe drone is already in auto-land mode!')





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

#!/usr/bin/env python3




#Import python libraries
import sys
import math
import asyncio
import numpy as np
from uav_mavsdk import UAV




async def main():

	uav = UAV(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
	await uav.connect()
	await uav.start_offboard_mission()

	pos = uav.ekf.pos + np.array([[2],[1],[-1]])
	yaw = uav.ekf.att_euler[2] + math.radians(10)
	await uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[-0.5],[-1.5],[-0.5]])
	yaw = uav.ekf.att_euler[2] + math.radians(60)
	await uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[-1.5],[1],[-0.5]])
	yaw = uav.ekf.att_euler[2] - math.radians(45)
	await uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[1],[-1],[1]])
	yaw = uav.ekf.att_euler[2] - math.radians(20)
	await uav.set_pos_yaw(pos, yaw, 6)
	
	pos = uav.ekf.pos + np.array([[1],[1.5],[0.6]])
	yaw = uav.ekf.att_euler[2] + math.radians(35)
	await uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[-2],[-1],[-1]])
	yaw = uav.ekf.att_euler[2] - math.radians(60)
	await uav.set_pos_yaw(pos, yaw, 6)

	await uav.auto_land()




if __name__ == "__main__":
	asyncio.get_event_loop().run_until_complete(main())

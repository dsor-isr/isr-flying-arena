#!/usr/bin/env python3




#Import python libraries
import sys
import math
import rospy
import numpy as np
from uav_mavros import UAV




if __name__ == "__main__":

	rospy.init_node('position_setpoints', anonymous=True)

	uav = UAV(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
	uav.start_offboard_mission()

	pos = uav.ekf.pos + np.array([[2],[1],[-1]])
	yaw = uav.ekf.att_euler[2] + math.radians(10)
	uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[-0.5],[-1.5],[-0.5]])
	yaw = uav.ekf.att_euler[2] + math.radians(60)
	uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[-1.5],[1],[-0.5]])
	yaw = uav.ekf.att_euler[2] - math.radians(45)
	uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[1],[-1],[1]])
	yaw = uav.ekf.att_euler[2] - math.radians(20)
	uav.set_pos_yaw(pos, yaw, 6)
	
	pos = uav.ekf.pos + np.array([[1],[1.5],[0.6]])
	yaw = uav.ekf.att_euler[2] + math.radians(35)
	uav.set_pos_yaw(pos, yaw, 6)

	pos = uav.ekf.pos + np.array([[-2],[-1],[-1]])
	yaw = uav.ekf.att_euler[2] - math.radians(60)
	uav.set_pos_yaw(pos, yaw, 6)

	uav.auto_land()

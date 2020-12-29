#!/usr/bin/env python3




#Import python libraries
import sys
import math
import rospy
import numpy as np
from uav_mavros import UAV
from math import cos, sin, asin, atan




if __name__ == "__main__":

	rospy.init_node('lissajous', anonymous=True)

	uav = UAV(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
	uav.start_offboard_mission()

	pos = np.array([[0],[1.6],[-0.6]]); yaw = 0
	uav.set_pos_yaw(pos, yaw, 6)

	t0 = rospy.get_time()
	t = rospy.get_time()-t0
	while(t<60):
		pos = np.array([[2.6*sin(2*0.55*t)], [1.6*sin(3*0.55*t+np.pi/2)], [-1.2+0.6*cos(0.55*t)]])
		yaw += math.radians(4/30) if t<40 else -math.radians(16/30)
		uav.set_pos_yaw(pos, yaw, 1/30)
		t = rospy.get_time()-t0

	uav.set_pos_yaw(pos, yaw, 1)
	uav.auto_land()

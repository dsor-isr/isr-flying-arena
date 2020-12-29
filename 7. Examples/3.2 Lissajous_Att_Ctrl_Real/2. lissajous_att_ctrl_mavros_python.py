#!/usr/bin/env python3





#Import python libraries
import sys
import math
import rospy
import threading
import numpy as np
from uav_mavros import UAV
from scipy.linalg import norm
from std_msgs.msg import Header
from math import cos, sin, asin, atan
from geometry_msgs.msg import PoseStamped





#Global variable shared by multiple threads
p_ref = np.array([[0],[1.6],[-0.6]])





def controller(u):

	g = 9.8066
	e3 = np.array([[0], [0], [1]])
	
	TRe3 = uav.info.mass*g*e3 - u 
	T = norm(TRe3)
	Re3 = TRe3/T
	att = np.array([[asin(-Re3[1][0])], [atan(Re3[0][0]/Re3[2][0])], [0]])

	return (att, T)





def send_refs_offboard_logger():

	def ned_to_enu_inertial(v):
		return np.array([[v[1][0]],[v[0][0]],[-v[2][0]]])

	global p_ref

	pub = rospy.Publisher(sys.argv[1]+'/mavros/waypoints/position', PoseStamped, queue_size=1) 

	pose = PoseStamped() 
	pose.header = Header()

	while(True):
		p_logs = p_ref
		rate=rospy.Rate(80); rate.sleep()
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = ned_to_enu_inertial(p_logs)
		pub.publish(pose)





if __name__ == "__main__":

	rospy.init_node('lissajous', anonymous=True)

	uav = UAV(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
	threading.Thread(target = send_refs_offboard_logger, daemon=True).start()
	uav.start_offboard_mission()

	uav.set_pos_yaw(p_ref, 0, 6)

	kp = np.diag([float(sys.argv[7]), float(sys.argv[7]), float(sys.argv[10])])
	kd = np.diag([float(sys.argv[8]), float(sys.argv[8]), float(sys.argv[11])])
	ki = np.diag([float(sys.argv[9]), float(sys.argv[9]), float(sys.argv[12])])
	integral = np.array([[0.0], [0.0], [0.0]])
	
	t0 = rospy.get_time()
	t = rospy.get_time()-t0

	while(t<60):
		p = uav.ekf.pos; v = uav.ekf.vel

		p_ref = np.array([[2.6*sin(2*0.55*t)], [1.6*sin(3*0.55*t+np.pi/2)], [-1.2+0.6*cos(0.35*t)]])
		v_ref = np.array([[2*0.55*2.6*cos(2*0.55*t)], [3*0.55*1.6*cos(3*0.55*t+np.pi/2)], [-0.35*0.6*sin(0.35*t)]])		
		a_ref = np.array([[-2*0.55*2*0.55*2.6*sin(2*0.55*t)], [-3*0.55*3*0.55*1.6*sin(3*0.55*t+np.pi/2)], [-0.35*0.35*0.6*cos(0.35*t)]])

		u = -np.dot(kp, (p-p_ref)) -np.dot(kd, (v-v_ref)) -np.dot(ki, integral) + uav.info.mass*a_ref
		att, T = controller(u)
		integral += (p-p_ref)*1/50

		uav.set_att_thrust(att, 'euler', T, 50)
		t = rospy.get_time()-t0

	uav.set_pos_yaw(p_ref, 0, 2)
	uav.auto_land()

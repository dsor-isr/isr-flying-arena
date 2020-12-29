#!/usr/bin/env python3





# Import python libraries
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





# Global variables shared by the multiple threads
ang = None
ang2 = None
p_ref = None
a_ref = None
pijd = None
vijd = None
aijd = None
land = False





def controller(u):

	g = 9.8066
	e3 = np.array([[0], [0], [1]])
	
	TRe3 = uav.info.mass*g*e3 - u 
	T = norm(TRe3)
	Re3 = TRe3/T
	att = np.array([[asin(-Re3[1][0])], [atan(Re3[0][0]/Re3[2][0])], [0]])

	return (att, T)





def ref_param():

	global ang, ang2, p_ref, a_ref, pijd, vijd, aijd, land

	t0=rospy.get_time(); t=rospy.get_time()-t0
	while(t<15):
		p_ref = np.array([[0.8], [0.0], [-1.25]])		
		a_ref = np.array([[0.0], [0.0], [ 0.00]])
		pijd = np.array([[[1.5*sin(ang)],[1.5*cos(ang)],[0.0]], [[1.5*sin(ang)-1.5*sin(ang2)],[1.5*cos(ang)-1.5*cos(ang2)],[0.0]]]) 
		vijd = np.array([[[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]]])  	
		aijd = np.array([[[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]]])
		rate = rospy.Rate(80); rate.sleep()
		t=rospy.get_time()-t0

	t0=rospy.get_time(); t=rospy.get_time()-t0
	while(t<35):
		p_ref = np.array([[ 0.8*cos(t/2)],[-0.300*sin(t/2)],[-1.25]])		
		a_ref = np.array([[-0.2*cos(t/2)],[ 0.075*sin(t/2)],[ 0.00]])
		pijd = np.array([[[1.5*sin(t/2+ang)],[1.5*cos(t/2+ang)],[0.0]], [[1.5*sin(t/2+ang)-1.5*sin(t/2+ang2)],[1.5*cos(t/2+ang)-1.5*cos(t/2+ang2)],[0.0]]]) 
		vijd = np.array([[[0.75*cos(t/2+ang)],[-0.75*sin(t/2+ang)],[0.0]], [[0.75*cos(t/2+ang)-0.75*cos(t/2+ang2)],[-0.75*sin(t/2+ang)+0.75*sin(t/2+ang2)],[0.0]]])
		aijd = np.array([[[-0.375*sin(t/2+ang)],[-0.375*cos(t/2+ang)],[0.0]], [[-0.375*sin(t/2+ang)+0.375*sin(t/2+ang2)],[-0.375*cos(t/2+ang)+0.375*cos(t/2+ang2)],[0.0]]])
		rate = rospy.Rate(80); rate.sleep()
		t=rospy.get_time()-t0

	land=True





def send_refs_offboard_logger():

	def ned_to_enu_inertial(v):
		return np.array([[v[1][0]],[v[0][0]],[-v[2][0]]])

	global ang, ang2, p_ref, pijd

	ang = float(sys.argv[10])
	ang2 = ang - 4*np.pi/6
	p_ref = np.array([[0.8],[0],[-1.25]])
	pijd = np.array([[[1.5*sin(ang)],[1.5*cos(ang)],[0.0]], [[1.5*sin(ang)-1.5*sin(ang2)],[1.5*cos(ang)-1.5*cos(ang2)],[0.0]]]) 

	pub = rospy.Publisher(sys.argv[1]+'/mavros/waypoints/position', PoseStamped, queue_size=1) 

	pose = PoseStamped() 
	pose.header = Header()

	while(True):
		p_logs = p_ref + pijd[0]
		rate=rospy.Rate(80); rate.sleep()
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = ned_to_enu_inertial(p_logs)
		pub.publish(pose)





if __name__ == "__main__":

	rospy.init_node('follower_'+sys.argv[1], anonymous=True)

	uav = UAV(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
	threading.Thread(target=send_refs_offboard_logger, daemon=True).start()
	uav.start_offboard_mission()


	kp = float(sys.argv[7]); kd = float(sys.argv[8]); ki = float(sys.argv[9])
	integral = np.array([[[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]]])
	
	threading.Thread(target=ref_param, daemon=True).start()
	uav.set_pos_yaw(p_ref+pijd[0], 0, 5)

	while(not land):
		u = a_ref + aijd[1]

		for i in range(2):
			pij = -uav.sen.emu.rel_pos[i]; vij = -uav.sen.emu.rel_vel[i];
			u -= (np.dot(kp,(pij-pijd[i])) + np.dot(kd,(vij-vijd[i])) + np.dot(ki,integral[i]))
			integral[i]+=(pij-pijd[i])*1/50	

		att, T = controller(u)
		uav.set_att_thrust(att, 'euler', T, 50)

	uav.set_pos_yaw(p_ref+pijd[0], 0, 2)
	uav.auto_land()

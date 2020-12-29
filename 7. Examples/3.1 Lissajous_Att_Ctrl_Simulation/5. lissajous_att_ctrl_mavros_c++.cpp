#include <math.h>
#include <stdlib.h>
#include "uav_mavros.h"





void controller(double u[3][1], double att[3][1], double *T, double mass){

	double g = 9.8066, T_aux;
	double Re3[3][1], att_aux[3][1];

	T_aux = sqrt(pow(-u[0][0], 2) + pow(-u[1][0], 2) + pow(mass*g-u[2][0], 2));

	Re3[0][0]=-u[0][0]/T_aux; Re3[1][0]=-u[1][0]/T_aux; Re3[2][0]=(mass*g-u[2][0])/T_aux;

	att_aux[0][0]=asin(-Re3[1][0]); att_aux[1][0]=atan(Re3[0][0]/Re3[2][0]) ; att_aux[2][0]=0;

	*T = T_aux;
	memcpy(att, att_aux, sizeof(att_aux));

	return;
}





int main(int argc, char **argv){

	int i;
	double t0, t, T;
	double kp[3][3], kd[3][3], ki[3][3], integral[3][1];
	double p[3][1], v[3][1], p_ref[3][1], v_ref[3][1], a_ref[3][1], att[3][1], att_q[4][1], u[3][1];

	ros::init(argc, argv, "mavros_cpp_" + (string) argv[1]); 

	UAV uav(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
	uav.start_offboard_mission();

	p_ref[0][0]=0.0; p_ref[1][0]=1.6; p_ref[2][0]=-0.6;
	uav.set_pos_yaw(p_ref, 0.0, 6.0);

	kp[0][0]=stod(argv[7]); kp[1][1]=stod(argv[7]); kp[2][2]=stod(argv[10]);
	kd[0][0]=stod(argv[8]); kd[1][1]=stod(argv[8]); kd[2][2]=stod(argv[11]);
	ki[0][0]=stod(argv[9]); ki[1][1]=stod(argv[9]); ki[2][2]=stod(argv[12]);
	integral[0][0]=0.0; integral[1][0]=0.0; integral[2][0]=0.0;

	t0 = ros::Time::now().toSec();
	t = ros::Time::now().toSec()-t0;

	while(t<60){
		memcpy(p, uav.ekf.pos, sizeof(uav.ekf.pos));
		memcpy(v, uav.ekf.vel, sizeof(uav.ekf.vel));

		p_ref[0][0]=2.6*sin(2.0*0.55*t); p_ref[1][0]=1.6*sin(3.0*0.55*t+M_PI/2.0); p_ref[2][0]=-1.2+0.6*cos(0.35*t);
		v_ref[0][0]=2.0*0.55*2.6*cos(2.0*0.55*t); v_ref[1][0]=3.0*0.55*1.6*cos(3.0*0.55*t+M_PI/2.0); v_ref[2][0]=-0.35*0.6*sin(0.35*t);
		a_ref[0][0]=-2.0*0.55*2.0*0.55*2.6*sin(2.0*0.55*t); a_ref[1][0]=-3.0*0.55*3.0*0.55*1.6*sin(3.0*0.55*t+M_PI/2.0); a_ref[2][0]=-0.35*0.35*0.6*cos(0.35*t);

		for (i=0; i<=2; i++)
			u[i][0] = -kp[i][i]*(p[i][0]-p_ref[i][0]) -kd[i][i]*(v[i][0]-v_ref[i][0]) -ki[i][i]*integral[i][0] + uav.info.mass*a_ref[i][0];

		controller(u, att, &T, uav.info.mass);

		for (i=0; i<=2; i++)
			integral[i][0] = integral[i][0] + (p[i][0]-p_ref[i][0])*1.0/50.0; 

		uav.set_att_thrust(att, att_q, "euler", T, 50.0);
		t = ros::Time::now().toSec()-t0;
	}

	uav.set_pos_yaw(p_ref, 0.0, 2.0);
	uav.auto_land();
	return 0;
}

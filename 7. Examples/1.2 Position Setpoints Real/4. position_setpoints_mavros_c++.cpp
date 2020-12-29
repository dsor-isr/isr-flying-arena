#include <math.h>
#include <stdlib.h>
#include "uav_mavros.h"




void add_two_arrays(double pos[3][1], double a1[3][1], double a2[3][1]){

	int i;
	double res[3][1];

	for (i=0; i<=2; i++)
		res[i][0] = a1[i][0] + a2[i][0];

	memcpy(pos, res, sizeof(res));
	return;
}




int main(int argc, char **argv){

	double pos[3][1], delta_pos[3][1];
	double yaw;

	ros::init(argc, argv, "mavros_cpp_" + (string) argv[1]); 

	UAV uav(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
	uav.start_offboard_mission();

	delta_pos[0][0]=2.0; delta_pos[1][0]=1.0; delta_pos[2][0]=-1.0;
	add_two_arrays(pos, uav.ekf.pos, delta_pos);
	yaw = uav.ekf.att_euler[2][0] + 10.0*M_PI/180;
	uav.set_pos_yaw(pos, yaw, 6);

	delta_pos[0][0]=-0.5; delta_pos[1][0]=-1.5; delta_pos[2][0]=-0.5;
	add_two_arrays(pos, uav.ekf.pos, delta_pos);
	yaw = uav.ekf.att_euler[2][0] + 60.0*M_PI/180;
	uav.set_pos_yaw(pos, yaw, 6);

	delta_pos[0][0]=-1.5; delta_pos[1][0]=1.0; delta_pos[2][0]=-0.5;
	add_two_arrays(pos, uav.ekf.pos, delta_pos);
	yaw = uav.ekf.att_euler[2][0] - 45.0*M_PI/180;
	uav.set_pos_yaw(pos, yaw, 6);

	delta_pos[0][0]=1.0; delta_pos[1][0]=-1.0; delta_pos[2][0]=1.0;
	add_two_arrays(pos, uav.ekf.pos, delta_pos);
	yaw = uav.ekf.att_euler[2][0] - 20.0*M_PI/180;
	uav.set_pos_yaw(pos, yaw, 6);

	delta_pos[0][0]=1.0; delta_pos[1][0]=1.5; delta_pos[2][0]=0.6;
	add_two_arrays(pos, uav.ekf.pos, delta_pos);
	yaw = uav.ekf.att_euler[2][0] + 35.0*M_PI/180;
	uav.set_pos_yaw(pos, yaw, 6);

	delta_pos[0][0]=-2.0; delta_pos[1][0]=-1.0; delta_pos[2][0]=-1.0;
	add_two_arrays(pos, uav.ekf.pos, delta_pos);
	yaw = uav.ekf.att_euler[2][0] - 60.0*M_PI/180;
	uav.set_pos_yaw(pos, yaw, 6);

	uav.auto_land();
	return 0;
}

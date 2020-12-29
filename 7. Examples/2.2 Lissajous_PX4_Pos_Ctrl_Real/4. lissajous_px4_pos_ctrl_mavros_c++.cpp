#include <math.h>
#include <stdlib.h>
#include "uav_mavros.h"




int main(int argc, char **argv){

	double pos[3][1];
	double yaw, t0, t;

	ros::init(argc, argv, "mavros_cpp_" + (string) argv[1]); 

	UAV uav(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
	uav.start_offboard_mission();

	pos[0][0]=0.0; pos[1][0]=1.6; pos[2][0]=-0.6;
	yaw = 0.0;
	uav.set_pos_yaw(pos, yaw, 6);

	t0 = ros::Time::now().toSec();
	t = ros::Time::now().toSec()-t0;

	while(t<60){
		pos[0][0]=2.6*sin(2*0.55*t); pos[1][0]=1.6*sin(3*0.55*t+M_PI/2); pos[2][0]=-1.2+0.6*cos(0.55*t);
		(t<40) ? yaw=yaw+4.0/30.0*M_PI/180.0 : yaw=yaw-16.0/30.0*M_PI/180.0;
		uav.set_pos_yaw(pos, yaw, 1.0/30.0);
		t = ros::Time::now().toSec()-t0;
	}

	uav.set_pos_yaw(pos, yaw, 1);
	uav.auto_land();
	return 0;
}

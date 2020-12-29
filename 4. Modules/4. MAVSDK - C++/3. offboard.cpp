#include "uav_mavsdk.h"





// Auxiliary functions of this module.
float rad_to_deg(double p);
double normalize_thrust(double thrust_newtons, string thrust_curve, double vel);





UAV::UAV(string drone_udp, string mass, string radius, string height, string num_rotors, string thrust_curve){
	/*****
	Constructor of the UAV class. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
	Awaits until the connection with the drone is established.

	Parameters
	----------
	drone_udp : str
		UDP address of the PX4 autopilot.
	mass : str
		Mass of the drone.
	radius : str
		Radius of the vehicle.
	height : str
		Height of the drone.
	num_rotors : str
		Number of rotors of the drone.
	thrust_curve : str
		Thrust curve of the vehicle.
	*****/
	info.drone_udp = "udp://:"+drone_udp;
	info.mass = stod(mass);
	info.radius = stod(radius);
	info.height = stod(height);
	info.num_rotors = stoi(num_rotors);
	info.thrust_curve = thrust_curve;

	printf("\nConnecting to the drone...");
	dc.add_any_connection(info.drone_udp);
	while(!dc.is_connected()){
		dc.add_any_connection(info.drone_udp);
		sleep_for(seconds(2));
	}
	printf("\nConnection established!");
	
	thread background_thread(&UAV::init_telemetry, this);
	background_thread.detach();
	sleep_for(seconds(5));

	action = make_shared<Action>(system);
	offboard = make_shared<Offboard>(system);
}





void OFFBOARD::arm_drone(){
	/*****
	Arms the drone, if it is not already armed.
	*****/
	if (info.is_armed == false){
		printf("\n\nArming drone...");
		const Action::Result arm_result = action->arm();
		if (arm_result == Action::Result::SUCCESS){
			sleep_for(seconds(1));
			if (info.is_armed == true)
				printf("\nDrone armed!");
			else
				printf("\nUnable to arm the drone.");
		}
		else
			printf("\nUnable to arm the drone.");
	}
	else
		printf("\n\nThe drone is already armed!");
}





void OFFBOARD::start_offboard_mode(){
	/*****
	Changes the flight mode of the PX4 autopilot of the drone to offboard.
	*****/
	if (info.flight_mode.compare("OFFBOARD") != 0){
		set_pos_yaw(ekf.pos, ekf.att_euler[2][0], 1.0);
		printf("\n\nChanging to offboard mode...");
		Offboard::Result offboard_result = offboard->start();
		if (offboard_result == Offboard::Result::SUCCESS){
			sleep_for(seconds(1));
			if (info.flight_mode.compare("OFFBOARD") == 0)			
				printf("\nOffboard mode activated!");
			else
				printf("\nUnable to change to offboard mode.");
		}
		else
			printf("\nUnable to change to offboard mode.");
	}
	else
		printf("\n\nThe offboard mode is already set!");
}





void OFFBOARD::start_offboard_mission(){
	/*****
	Makes the vehicle ready for an offboard experiment by arming it and by changing the flight mode of its PX4 autopilot to offboard.
	*****/
	start_offboard_mode();
	arm_drone();
}





void OFFBOARD::set_pos_yaw(double pos[3][1], double yaw, double time){
	/*****
	Offboard method that sends position and yaw references to the PX4 autopilot of the the drone.

	Parameters
	----------
	pos : array of doubles with dimensions 3x1
		Desired position for the drone, in local NED coordinates.
	yaw : double
		Desired yaw for the vehicle, in radians.
	time: double
		Time, in seconds, during which the selected position and yaw references will be sent to the PX4 autopilot.
	*****/
	double t=0.0;

	while(t<time){
		offboard->set_position_ned({(float) pos[0][0], (float) pos[1][0], (float) pos[2][0], rad_to_deg(yaw)});	
		sleep_for(milliseconds(20));
		t=t+1.0/50.0;
	}
}





void OFFBOARD::set_vel_yaw(double vel[3][1], double yaw, double freq){
	/*****
	Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.

	Parameters
	----------
	vel : array of doubles with dimensions 3x1
		Desired linear velocity for drone, in local NED coordinates.
	yaw : double
		Desired yaw for the vehicle, in radians.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	int time = 1000.0/freq;
	offboard->set_velocity_ned({(float) vel[0][0], (float) vel[1][0], (float) vel[2][0], rad_to_deg(yaw)});	
	sleep_for(milliseconds(time));
}





void OFFBOARD::set_vel_body_yaw_rate(double vel_body[3][1], double yaw_rate, double freq){
	/*****
	Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.

	Parameters
	----------
	vel_body : array of doubles with dimensions 3x1
		Desired linear velocity for the drone, in body NED coordinates.
	yaw_rate : double
		Desired yaw rate for the vehicle, in radians per second.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	int time = 1000.0/freq;
	offboard->set_velocity_body({(float) vel_body[0][0], (float) vel_body[1][0], (float) vel_body[2][0], rad_to_deg(yaw_rate)});	
	sleep_for(milliseconds(time));
}





void OFFBOARD::set_att_thrust(double att_euler[3][1], double att_q[4][1], string att_type, double thrust, double freq){
	/*****
	Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.

	Converts the thrust from newtons to a normalized value between 0 and 1 through mathematical expression of the thrust curve of the vehicle.

	Parameters
	----------
	att_euler : array of doubles with dimensions 3x1
		Desired attitude for the vehicle, expressed in euler angles.
	att_q : array of doubles with dimensions 4x1
		Desired attitude for the vehicle, expressed in a quaternion.
	att_type : str
		Must be equal to either 'euler' or 'quaternion'. Specifies the format of the desired attitude.
	thrust : double
		Desired thrust value in newtons.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	(void) att_q; (void) att_type;
	int time = 1000.0/freq;
	double thr = normalize_thrust(thrust, info.thrust_curve, sqrt(pow(ekf.vel[0][0],2)+pow(ekf.vel[1][0],2)+pow(ekf.vel[2][0],2)));
	offboard->set_attitude({rad_to_deg(att_euler[0][0]), rad_to_deg(att_euler[1][0]), rad_to_deg(att_euler[2][0]), (float) thr});	
	sleep_for(milliseconds(time));
}





void OFFBOARD::set_ang_vel_thrust(double ang_vel[3][1], double thrust, double freq){
	/*****
	Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.

	Converts the thrust from newtons to a normalized value between 0 and 1 through the mathematical expression of the thrust curve of the vehicle.

	Parameters
	----------
	ang_vel : array of doubles with dimensions 3x1
		Desired angular velocity for the drone.
	thrust : double
		Desired thrust value in newtons.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	int time = 1000.0/freq;
	double thr = normalize_thrust(thrust, info.thrust_curve, sqrt(pow(ekf.vel[0][0],2)+pow(ekf.vel[1][0],2)+pow(ekf.vel[2][0],2)));
	offboard->set_attitude_rate({rad_to_deg(ang_vel[0][0]), rad_to_deg(ang_vel[1][0]), rad_to_deg(ang_vel[2][0]), (float) thr});	
	sleep_for(milliseconds(time));
}





void OFFBOARD::disarm_drone(){
	/*****
	Disarms the vehicle, if it is not already disarmed.
	*****/
	if (info.is_armed == true){
		printf("\n\nDisarming drone...");
		const Action::Result disarm_result = action->disarm();
		if (disarm_result == Action::Result::SUCCESS){
			sleep_for(seconds(1));
			if (info.is_armed == false)
				printf("\nDrone disarmed!");
			else
				printf("\nUnable to disarm the drone.");		
		}
		else
			printf("\nUnable to disarm the drone.");		
	}
	else
		printf("\n\nThe drone is already disarmed!");
}





void OFFBOARD::auto_land(){	
	/*****
	Lands the drone, changing its flight mode to auto-land.
	*****/
	if (info.flight_mode.compare("LAND") != 0){
		printf("\n\nChanging to auto-land mode...");
		const Action::Result land_result = action->land();
		if (land_result == Action::Result::SUCCESS){
			printf("\nAuto-land mode activated!\n\n");
			sleep_for(seconds(1));
			while(true){
				if (info.is_landed==true)
					break;
			}
		}
		else
			printf("\nUnable to change to auto-land mode.");
	}
	else
		printf("\n\nThe drone is already in auto-land mode!");
}





float rad_to_deg(double p){
	/*****
	Converts a parameter from radians to degrees and from double to float.

	Parameters
	----------
	p : double
		Parameter in radians.
	
	Returns
	-------
	float
		Parameter in degrees.
	*****/
	float pd = p*180.0/M_PI;

	return pd;
}





double normalize_thrust(double thrust_newtons, string thrust_curve, double vel){
	/*****
	Converts the thrust in newtons to a normalized value between 0 and 1 through the mathematical expression of the thrust curve of the drone.

	Parameters
	----------
	thrust_newtons : double
		Desired thrust value in newtons.
	thrust_curve: str
		Thrust curve of the vehicle.
	v : double
		Norm of the linear velocity of the vehicle.
	
	Returns
	-------
	double
		Normalized thrust, between 0 and 1.
	*****/
	double norm_thrust;

	if (thrust_newtons <= 0)
		return 0.0;
	else if (thrust_curve.compare("iris") == 0)
		norm_thrust = (thrust_newtons/(1.0-vel/25.0) - 1.52*9.80665)/(2.0*34.068*0.561 + 7.1202) + 0.561;
	else if (thrust_curve.compare("intel_aero") == 0)
		norm_thrust = (tan((thrust_newtons-10.37)/8.84)+1.478)/2.955;
	else if (thrust_curve.compare("snap_dragon") == 0)
		norm_thrust = 0.1377*exp(0.02976*thrust_newtons)*sqrt(thrust_newtons) + 0.003759*thrust_newtons - 0.05973;

	if (norm_thrust <= 0)
		return 0.0;
	else if (norm_thrust >= 1)
		return 1.0;
	else
		return norm_thrust;
}

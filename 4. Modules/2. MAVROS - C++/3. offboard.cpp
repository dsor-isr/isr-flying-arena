#include "uav_mavros.h"





// Auxiliary functions of this module.
void ned_to_enu(double v_ned[3][1], double v_enu[3][1]);
void SI_quaternion_to_ROS_quaternion(double q_si[4][1], double q_ros[4]);
double normalize_thrust(double thrust_newtons, string thrust_curve, double vel);





UAV:: UAV(string drone_ns, string mass, string radius, string height, string num_rotors, string thrust_curve){
	/*****
	Constructor of the UAV class. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
	Awaits until the connection with the drone is established.

	Parameters
	----------
	drone_ns : str
		ROS namespace where the data from the PX4 autopilot and the MOCAP system is encapsulated.
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
	info.drone_ns = drone_ns;
	info.mass = stod(mass);
	info.radius = stod(radius);
	info.height = stod(height);
	info.num_rotors = stoi(num_rotors);
	info.thrust_curve = thrust_curve;

	thread background_thread(&UAV::init_telemetry, this);
	background_thread.detach();

	printf("\nConnecting to the drone...");
	while(!info.is_connected){
		ros::Rate rate_u(2.0); rate_u.sleep(); 
	}
	printf("\nConnection established!");
	ros::Rate rate_d(1.0/5.0); rate_d.sleep();
}





void OFFBOARD::arm_drone(){
	/*****
	Arms the drone, if it is not already armed.

	Makes a request, to the ROS service responsible for changing the armed state of the vehicle, to switch the arm value to True.
	*****/
	ros::ServiceClient arm_client;
	mavros_msgs::CommandBool arm;

	if (info.is_armed == false){
		arm_client = nh.serviceClient<mavros_msgs::CommandBool>(info.drone_ns+"/mavros/cmd/arming");
		arm.request.value = true;
		printf("\n\nArming drone...");
		if (arm_client.call(arm) && arm.response.success){
			ros::Rate rate(1.0); rate.sleep();			
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

	Makes a request, to the ROS service responsible for changing the flight mode, to switch it to OFFBOARD.
	*****/
	ros::ServiceClient set_offboard_client;
	mavros_msgs::SetMode offboard_mode;

	set_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(info.drone_ns+"/mavros/setpoint_position/local", 1);
	set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>(info.drone_ns+"/mavros/setpoint_raw/local", 1);
	set_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>(info.drone_ns+"/mavros/setpoint_raw/attitude", 1);
	set_ang_vel_pub = nh.advertise<mavros_msgs::AttitudeTarget>(info.drone_ns+"/mavros/setpoint_raw/attitude", 1);
	set_act_pub = nh.advertise<mavros_msgs::ActuatorControl>(info.drone_ns+"/mavros/actuator_control", 1);

	if (info.flight_mode.compare("OFFBOARD") != 0){
		set_offboard_client = nh.serviceClient<mavros_msgs::SetMode>(info.drone_ns+"/mavros/set_mode");
		set_pos_yaw(ekf.pos, ekf.att_euler[2][0], 1.0);
		offboard_mode.request.custom_mode = "OFFBOARD";
		printf("\n\nChanging to offboard mode...");
		if (set_offboard_client.call(offboard_mode) && offboard_mode.response.mode_sent){
			ros::Rate rate(1.0); rate.sleep();
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

	Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS.

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
	ros::Rate r(50.0);
	std_msgs::Header h;
	double pos_enu[3][1];
	geometry_msgs::PoseStamped msg;
	geometry_msgs::Quaternion q_msg;

	while(t<time){
		h.stamp = ros::Time::now();
		msg.header = h;
		ned_to_enu(pos, pos_enu);
		msg.pose.position.x = pos_enu[0][0]; msg.pose.position.y = pos_enu[1][0]; msg.pose.position.z = pos_enu[2][0];
		tf2::Quaternion q_tf; q_tf.setRPY(0,0,-yaw); q_msg = tf2::toMsg(q_tf);
		msg.pose.orientation = q_msg;
		set_pos_pub.publish(msg);
		r.sleep();
		t=t+1.0/50.0;
	}
}





void OFFBOARD::set_vel_yaw(double vel[3][1], double yaw, double freq){
	/*****
	Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.

	Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS.

	Parameters
	----------
	vel : array of doubles with dimensions 3x1
		Desired linear velocity for drone, in local NED coordinates.
	yaw : double
		Desired yaw for the vehicle, in radians.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	ros::Rate r(freq);
	std_msgs::Header h;
	double vel_enu[3][1];
	mavros_msgs::PositionTarget msg;

	h.stamp = ros::Time::now();
	msg.header = h;
	msg.coordinate_frame = 1; msg.type_mask = 3015;
	ned_to_enu(vel, vel_enu);
	msg.velocity.x = vel_enu[0][0]; msg.velocity.y = vel_enu[1][0]; msg.velocity.z = vel_enu[2][0]; 
	msg.yaw = -yaw;
	set_vel_pub.publish(msg);
	r.sleep();
}





void OFFBOARD::set_vel_body_yaw_rate(double vel_body[3][1], double yaw_rate, double freq){
	/*****
	Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.

	Makes convertions from the body NED frame adopted for the ISR Flying Arena to the body ENU frame used by ROS.

	Parameters
	----------
	vel_body : array of doubles with dimensions 3x1
		Desired linear velocity for the drone, in body NED coordinates.
	yaw_rate : double
		Desired yaw rate for the vehicle, in radians per second.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	ros::Rate r(freq);
	std_msgs::Header h;
	double vel_enu[3][1];
	mavros_msgs::PositionTarget msg;

	h.stamp = ros::Time::now();
	msg.header = h;
	msg.coordinate_frame = 8; msg.type_mask = 1991;
	ned_to_enu(vel_body, vel_enu);
	msg.velocity.x = vel_enu[0][0]; msg.velocity.y = vel_enu[1][0]; msg.velocity.z = vel_enu[2][0]; 
	msg.yaw_rate = -yaw_rate;
	set_vel_pub.publish(msg);
	r.sleep();
}





void OFFBOARD::set_att_thrust(double att_euler[3][1], double att_q[4][1], string att_type, double thrust, double freq){
	/*****
	Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.

	Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS. 
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
	ros::Rate r(freq);
	std_msgs::Header h;
	mavros_msgs::AttitudeTarget msg;
	geometry_msgs::Quaternion q_msg;

	h.stamp = ros::Time::now();
	msg.header = h;

	if (att_type.compare("euler") == 0){
		double att_enu[3][1];
		ned_to_enu(att_euler, att_enu);
		tf2::Quaternion q_tf; q_tf.setRPY(att_enu[0][0],att_enu[1][0],att_enu[2][0]); q_msg = tf2::toMsg(q_tf);
	}
	else if (att_type.compare("quaternion") == 0){
		double att_q_ros[4];
		SI_quaternion_to_ROS_quaternion(att_q, att_q_ros);
		tf2::Quaternion q_tf(att_q_ros[0], att_q_ros[1], att_q_ros[2], att_q_ros[3]); q_msg = tf2::toMsg(q_tf);
	}	

	msg.orientation = q_msg;
	msg.thrust = normalize_thrust(thrust, info.thrust_curve, sqrt(pow(ekf.vel[0][0],2)+pow(ekf.vel[1][0],2)+pow(ekf.vel[2][0],2)));
	set_att_pub.publish(msg);
	r.sleep();
}





void OFFBOARD::set_ang_vel_thrust(double ang_vel[3][1], double thrust, double freq){
	/*****
	Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.

	Makes convertions from the local NED frame adopted for the ISR Flying Arena to the local ENU frame used by ROS.
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
	ros::Rate r(freq);
	std_msgs::Header h;
	double ang_vel_enu[3][1];
	mavros_msgs::AttitudeTarget msg;

	h.stamp = ros::Time::now();
	msg.header = h;
	msg.type_mask = 128; ned_to_enu(ang_vel, ang_vel_enu);
	msg.body_rate.x = ang_vel_enu[0][0]; msg.body_rate.y = ang_vel_enu[1][0]; msg.body_rate.z = ang_vel_enu[2][0];
	msg.thrust = normalize_thrust(thrust, info.thrust_curve, sqrt(pow(ekf.vel[0][0],2)+pow(ekf.vel[1][0],2)+pow(ekf.vel[2][0],2)));
	set_ang_vel_pub.publish(msg);
	r.sleep();
}





void OFFBOARD::set_act(int group, double output[8], double freq){
	/*****
	Offboard method that sets the values of the mixers and/or actuators of the vehicle.

	Parameters
	----------
	group: int
		Desired control group.
	output : array of doubles with size 8
		Desired output values for the mixers and/or actuators of the drone.
	freq : double
		Topic publishing frequency, in Hz.
	*****/
	ros::Rate r(freq);
	std_msgs::Header h;
	mavros_msgs::ActuatorControl msg;

	h.stamp = ros::Time::now();
	msg.header = h;
	msg.group_mix = group;
	msg.controls[0]=output[0]; msg.controls[1]=output[1]; msg.controls[2]=output[2]; msg.controls[3]=output[3]; 
	msg.controls[4]=output[4]; msg.controls[5]=output[5]; msg.controls[6]=output[6]; msg.controls[7]=output[7]; 
	set_act_pub.publish(msg);
	r.sleep();
}





void OFFBOARD::disarm_drone(){
	/*****
	Disarms the vehicle, if it is not already disarmed.

	Makes a request, to the ROS service responsible for changing the armed state of the vehicle, to switch the arm value to False.
	*****/
	ros::ServiceClient disarm_client;
	mavros_msgs::CommandBool arm;

	if (info.is_armed == true){
		disarm_client = nh.serviceClient<mavros_msgs::CommandBool>(info.drone_ns+"/mavros/cmd/arming");
		arm.request.value = false;
		printf("\n\nDisarming drone...");
		if (disarm_client.call(arm) && arm.response.success){
			ros::Rate rate(1.0); rate.sleep();
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
	ros::ServiceClient auto_land_client;
	mavros_msgs::SetMode auto_land_mode;

	if (info.flight_mode.compare("AUTO.LAND") != 0){
		auto_land_client = nh.serviceClient<mavros_msgs::SetMode>(info.drone_ns+"/mavros/set_mode");
		auto_land_mode.request.custom_mode = "AUTO.LAND";
		printf("\n\nChanging to auto-land mode...");
		if (auto_land_client.call(auto_land_mode) && auto_land_mode.response.mode_sent){
			ros::Rate rate(1.0); rate.sleep();
			if (info.flight_mode.compare("AUTO.LAND") == 0){	
				printf("\nAuto-land mode activated!\n\n");
				while(true){
					if (info.is_landed==true)
						break;
				}
			}
			else
				printf("\nUnable to change to auto-land mode.");
		}
		else
			printf("\nUnable to change to auto-land mode.");
	}
	else
		printf("\n\nThe drone is already in auto-land mode!");
}





void ned_to_enu(double v_ned[3][1], double v_enu[3][1]){
	/*****
	Converts a 3x1 physical variable from NED coordinates to ENU coordinates.

	Parameters
	----------
	v_ned : array of doubles with dimensions 3x1
		3x1 physical variable in NED coordinates.

	v_enu : array of doubles with dimensions 3x1
		Array that will store the 3x1 physical variable in ENU coordinates.
	*****/
	double v_aux[3][1] = {{v_ned[1][0]}, {v_ned[0][0]}, {-v_ned[2][0]}};
	memcpy(v_enu, v_aux, sizeof(v_aux));
}





void SI_quaternion_to_ROS_quaternion(double q_si[4][1], double q_ros[4]){
	/*****
	Converts a SI quaternion in NED coordinates to a ROS quaternion in ENU coordinates.

	Parameters
	----------
	q_si : array of double with dimensions 4x1
		Quaternion in the SI format and NED coordinates.

	q_ros : array of doubles of size 4
		Variable that will store the quaternion in the ROS format and ENU coordinates.
	*****/
	double q_aux[4] = {q_si[2][0], q_si[1][0], -q_si[3][0], q_si[0][0]};
	memcpy(q_ros, q_aux, sizeof(q_aux));
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

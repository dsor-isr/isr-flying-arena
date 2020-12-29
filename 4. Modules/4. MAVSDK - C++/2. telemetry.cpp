#include "uav_mavsdk.h"





void TELEMETRY::init_telemetry(){
	/*****
	Defines the callback methods responsible for updating each variable of the UAV class.
	*****/
	telemetry = make_shared<Telemetry>(system);

	telemetry->position_velocity_ned_async(bind(&TELEMETRY::update_position_and_velocity, this, _1));
	telemetry->odometry_async(bind(&TELEMETRY::update_velocity_body, this, _1));
	telemetry->attitude_quaternion_async(bind(&TELEMETRY::update_attitude_quaternion, this, _1));
	telemetry->attitude_euler_angle_async(bind(&TELEMETRY::update_attitude_euler, this, _1));
	telemetry->attitude_angular_velocity_body_async(bind(&TELEMETRY::update_angular_velocity, this, _1));
	telemetry->actuator_control_target_async(bind(&TELEMETRY::update_actuator, this, _1));
	telemetry->flight_mode_async(bind(&TELEMETRY::update_flight_mode, this, _1));
	telemetry->armed_async(bind(&TELEMETRY::update_status, this, _1));
	telemetry->in_air_async(bind(&TELEMETRY::update_landed, this, _1));
	telemetry->battery_async(bind(&TELEMETRY::update_battery, this, _1));
}





void TELEMETRY::update_position_and_velocity(Telemetry::PositionVelocityNED msg){
	/*****
	Updates the variables that store the position and linear velocity of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

	Parameters
	----------
	msg : Telemetry::PositionVelocityNED 
		MAVSDK message containing the extended Kalman filter output value for the position and velocity of the drone.
	*****/
	double pos_aux[3][1] = {{msg.position.north_m}, {msg.position.east_m}, {msg.position.down_m}};
	memcpy(ekf.pos, pos_aux, sizeof(ekf.pos));

	double vel_aux[3][1] = {{msg.velocity.north_m_s}, {msg.velocity.east_m_s}, {msg.velocity.down_m_s}};
	memcpy(ekf.vel, vel_aux, sizeof(ekf.vel));
}





void TELEMETRY::update_velocity_body(Telemetry::Odometry msg){
	/*****
	Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

	Parameters
	----------
	msg : Telemetry::Odometry
		MAVSDK message containing the extended Kalman filter output value for the velocity of the drone.
	*****/
	double vel_aux[3][1] = {{msg.velocity_body.x_m_s}, {msg.velocity_body.y_m_s}, {msg.velocity_body.z_m_s}};
	memcpy(ekf.vel_body, vel_aux, sizeof(ekf.vel_body));
}





void TELEMETRY::update_attitude_quaternion(Telemetry::Quaternion msg){
	/*****
	Updates the variable that stores the attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.

	Parameters
	----------
	msg : Telemetry::Quaternion
		MAVSDK message containing the extended Kalman filter output value for the attitude of the vehicle in quaternions.
	*****/
	double att_aux[4][1] = {{msg.w}, {msg.x}, {msg.y}, {msg.z}};
	memcpy(ekf.att_q, att_aux, sizeof(ekf.att_q));
}





void TELEMETRY::update_attitude_euler(Telemetry::EulerAngle msg){
	/*****
	Updates the variable that stores the attitude of the vehicle, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.

	Makes appropriate convertions to SI units since the MAVSDK library stores angles in degrees.

	Parameters
	----------
	msg : Telemetry::EulerAngle
		MAVSDK message containing the extended Kalman filter output value for the attitude of the vehicle in Euler angles.
	*****/
	double att_aux[3][1] = {{msg.roll_deg*M_PI/180}, {msg.pitch_deg*M_PI/180}, {msg.yaw_deg*M_PI/180}};
	memcpy(ekf.att_euler, att_aux, sizeof(ekf.att_euler));
}





void TELEMETRY::update_angular_velocity(Telemetry::AngularVelocityBody msg){
	/*****
	Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

	Parameters
	----------
	msg : Telemetry::AngularVelocityBody
		MAVSDK message containing the the extended Kalman filter output value for the angular velocity of the vehicle.
	*****/
	double ang_vel_aux[3][1] = {{msg.roll_rad_s}, {msg.pitch_rad_s}, {msg.yaw_rad_s}};
	memcpy(ekf.ang_vel, ang_vel_aux, sizeof(ekf.ang_vel));
}





void TELEMETRY::update_actuator(Telemetry::ActuatorControlTarget msg){
	/*****
	Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.

	Parameters
	----------
	msg : Telemetry::ActuatorControlTarget
		MAVSDK message containing the normalized values applied to the mixer and/or motors and servos of the vehicle.
	*****/	
	act.group = msg.group;

	double act_aux[8] = {msg.controls[0], msg.controls[1], msg.controls[2], msg.controls[3], msg.controls[4], msg.controls[5], msg.controls[6], msg.controls[7]};
	memcpy(act.output, act_aux, sizeof(act.output));
}





void TELEMETRY::update_flight_mode(Telemetry::FlightMode msg){
	/*****
	Updates the variable that stores the current flight mode of the PX4 autopilot of the vehicle. 

	Parameters
	----------
	msg : FlightMode 
		MAVSDK message containing the current flight mode of the PX4 autopilot
	*****/
	info.flight_mode = Telemetry::flight_mode_str(msg);
}





void TELEMETRY::update_status(bool msg){
	/*****
	Updates the variable that stores the armed state of the vehicle. 

	Parameters
	----------
	msg : bool 
		MAVSDK message containing the armed state of the vehicle.
	*****/
	info.is_armed = msg;
}





void TELEMETRY::update_landed(bool msg){
	/*****
	Updates the variable that stores the landed state of the drone.

	Parameters
	----------
	msg : bool 
		MAVSDK message containing the landed state of the drone.
	*****/
	info.is_landed = !msg;
}





void TELEMETRY::update_battery(Telemetry::Battery msg){
	/*****
	Updates the variable that stores the remaining battery percentage. 

	Parameters
	----------
	msg : BatteryState (from sensor_msgs) 
		MAVSDK message containing information about the battery of the drone.
	*****/
	info.battery = msg.remaining_percent*100;
}

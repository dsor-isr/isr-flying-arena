#include "uav_mavros.h"





// Auxiliary functions of this module.
void enu_to_ned(double v_enu[3][1], double v_ned[3][1]);
void ROS_quaternion_to_SI_quaternion(double q_ros[4], double q_si[4][1]);
void ROS_list_to_array(const std_msgs::Float64MultiArray::ConstPtr& msg, double rel_param[100][3][1]);





void TELEMETRY::init_telemetry(){
	/*****
	Manages topic subscriptions.

	Sets the topics subscribers and defines the methods responsible for handling each subscription.
	*****/
	using namespace ros;

	Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(info.drone_ns+"/mavros/local_position/pose", 10, &UAV::update_position, this);
	Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(info.drone_ns+"/mavros/local_position/velocity_local", 10, &UAV::update_velocity, this);
	Subscriber vel_body_sub = nh.subscribe<geometry_msgs::TwistStamped>(info.drone_ns+"/mavros/local_position/velocity_body", 10, &UAV::update_velocity_body, this);
	Subscriber att_sub = nh.subscribe<geometry_msgs::PoseStamped>(info.drone_ns+"/mavros/local_position/pose", 10, &UAV::update_attitude, this);
	Subscriber ang_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(info.drone_ns+"/mavros/local_position/velocity_body", 10, &UAV::update_angular_velocity, this);
	Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(info.drone_ns+"/mavros/imu/data", 10, &UAV::update_imu, this);
	Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField>(info.drone_ns+"/mavros/imu/mag", 10, &UAV::update_mag, this);
	Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>(info.drone_ns+"/mavros/high_freq_vision_pose/pose", 10, &UAV::update_mocap, this);
	Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(info.drone_ns+"/mavros/global_position/raw/fix", 10, &UAV::update_gps, this);
	Subscriber baro_pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>(info.drone_ns+"/mavros/imu/static_pressure", 10, &UAV::update_baro_pressure, this);
	Subscriber baro_temperature_sub = nh.subscribe<sensor_msgs::Temperature>(info.drone_ns+"/mavros/imu/temperature_imu", 10, &UAV::update_baro_temperature, this);
	Subscriber baro_altitude_sub = nh.subscribe<mavros_msgs::Altitude>(info.drone_ns+"/mavros/altitude", 10, &UAV::update_baro_altitude, this);
	Subscriber rel_pos_sub = nh.subscribe<std_msgs::Float64MultiArray>(info.drone_ns+"/mavros/relative_positions/list", 10, &UAV::update_relative_positions, this);
	Subscriber rel_vel_sub = nh.subscribe<std_msgs::Float64MultiArray>(info.drone_ns+"/mavros/relative_velocities/list", 10, &UAV::update_relative_velocities, this);
	Subscriber act_sub = nh.subscribe<mavros_msgs::ActuatorControl>(info.drone_ns+"/mavros/target_actuator_control", 10, &UAV::update_actuator, this);
	Subscriber status_sub = nh.subscribe<mavros_msgs::State>(info.drone_ns+"/mavros/state", 10, &UAV::update_status, this);
	Subscriber landed_sub = nh.subscribe<mavros_msgs::ExtendedState>(info.drone_ns+"/mavros/extended_state", 10, &UAV::update_landed, this);
	Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>(info.drone_ns+"/mavros/battery", 10, &UAV::update_battery, this);

	MultiThreadedSpinner spinner(3);
	spinner.spin();
}





void TELEMETRY::update_position(const geometry_msgs::PoseStamped::ConstPtr& msg){
	/*****
	Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

	Makes appropriate convertions to the local NED frame since the ROS message stores the position of the drone in the local ENU frame.

	Parameters
	----------
	msg : PoseStamped (from geometry_msgs) 
		ROS message containing the extended Kalman filter output value for the position of the drone.
	*****/
	double pos_aux[3][1] = {{msg->pose.position.x}, {msg->pose.position.y}, {msg->pose.position.z}};
	enu_to_ned(pos_aux, ekf.pos);
}





void TELEMETRY::update_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg){
	/*****
	Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

	Makes appropriate convertions to the local NED frame since the ROS message stores the velocity of the drone in the local ENU frame.

	Parameters
	----------
	msg : TwistStamped (from geometry_msgs)
		ROS message containing the extended Kalman filter output value for the velocity of the drone.
	*****/
	double vel_aux[3][1] = {{msg->twist.linear.x}, {msg->twist.linear.y}, {msg->twist.linear.z}};
	enu_to_ned(vel_aux, ekf.vel);
}





void TELEMETRY::update_velocity_body(const geometry_msgs::TwistStamped::ConstPtr& msg){
	/*****
	Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.

	Makes appropriate convertions to the body NED frame since the ROS message stores the velocity of the drone in the body ENU frame.

	Parameters
	----------
	msg : TwistStamped (from geometry_msgs)
		ROS message containing the extended Kalman filter output value for the velocity of the drone.
	*****/
	double vel_aux[3][1] = {{msg->twist.linear.x}, {msg->twist.linear.y}, {msg->twist.linear.z}};
	enu_to_ned(vel_aux, ekf.vel_body);
}





void TELEMETRY::update_attitude(const geometry_msgs::PoseStamped::ConstPtr& msg){
	/*****
	Updates the variables that store the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

	Makes appropriate convertions to local NED frame since the ROS message stores the attitude of the drone in the local ENU frame.

	Parameters
	----------
	msg : PoseStamped (from geometry_msgs) 
		ROS message containing the extended Kalman filter output value for the attitude of the vehicle.
	*****/
	double ros_q[4] = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
	ROS_quaternion_to_SI_quaternion(ros_q, ekf.att_q);

	double roll, pitch, yaw;
	tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);  
	tf2::Matrix3x3 m(q); m.getRPY(roll, pitch, yaw); 
	double att_aux[3][1] = {{roll},{pitch},{yaw}};
	enu_to_ned(att_aux, ekf.att_euler);
}





void TELEMETRY::update_angular_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg){
	/*****
	Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.

	Makes appropriate convertions to the local NED frame since the ROS message stores the angular velocity of the drone in the local ENU frame.

	Parameters
	----------
	msg : TwistStamped (from geometry_msgs)
		ROS message containing the the extended Kalman filter output value for the angular velocity of the vehicle.
	*****/
	double ang_vel_aux[3][1] = {{msg->twist.angular.x}, {msg->twist.angular.y}, {msg->twist.angular.z}};
	enu_to_ned(ang_vel_aux, ekf.ang_vel);
}





void TELEMETRY::update_imu(const sensor_msgs::Imu::ConstPtr& msg){
	/*****
	Updates the variables that store the linear acceleration and the angular velocity of the drone measured by the IMU.

	Makes appropriate convertions to the body NED frame since the ROS message stores the IMU data in the body ENU frame.

	Parameters
	----------
	msg : Imu (from sensor_msgs)
		ROS message containing the IMU measurements for the linear acceleration and angular velocity of the vehicle.
	*****/
	double acc_body_aux[3][1] = {{msg->linear_acceleration.x}, {msg->linear_acceleration.y}, {msg->linear_acceleration.z}};
	enu_to_ned(acc_body_aux, sen.imu.acc_body);

	double ang_vel_aux[3][1] = {{msg->angular_velocity.x}, {msg->angular_velocity.y}, {msg->angular_velocity.z}};
	enu_to_ned(ang_vel_aux, sen.imu.ang_vel);
}





void TELEMETRY::update_mag(const sensor_msgs::MagneticField::ConstPtr& msg){
	/*****
	Updates the variable that stores the magnetic field vector, in body NED coordinates, measured by the IMU.

	Makes appropriate convertions to the body NED frame since the ROS message stores the magnetic field vector in the body ENU frame.

	Parameters
	----------
	msg : MagneticField (from sensor_msgs)
		ROS message containing the IMU measurements for the magnetic field vector.
	*****/
	double mag_aux[3][1] = {{msg->magnetic_field.x}, {msg->magnetic_field.y}, {msg->magnetic_field.z}};
	enu_to_ned(mag_aux, sen.imu.mag);
}





void TELEMETRY::update_mocap(const geometry_msgs::PoseStamped::ConstPtr& msg){
	/*****
	Updates the variables that store the position and attitude of the drone provided by the motion capture system.

	Makes appropriate convertions to the body NED frame since the ROS message stores the MOCAP pose in the body ENU frame.

	Parameters
	----------
	msg : PoseStamped (from geometry_msgs)
		ROS message containing the pose of the vehicle provided by the motion capture system.
	*****/
	double mocap_pos_aux[3][1] = {{msg->pose.position.x}, {msg->pose.position.y}, {msg->pose.position.z}};
	enu_to_ned(mocap_pos_aux, sen.mocap.pos);

	double ros_q[4] = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
	ROS_quaternion_to_SI_quaternion(ros_q, sen.mocap.att_q);

	double roll, pitch, yaw;
	tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);  
	tf2::Matrix3x3 m(q); m.getRPY(roll, pitch, yaw); 
	double att_aux[3][1] = {{roll},{pitch},{yaw}};
	enu_to_ned(att_aux, sen.mocap.att_euler);
}





void TELEMETRY::update_gps(const sensor_msgs::NavSatFix::ConstPtr& msg){
	/*****
	Updates the variable that stores the raw measurements provided by the GPS sensor.

	Makes appropriate convertions to SI units since the ROS message stores the latitude and longitude in degrees.

	Parameters
	----------
	msg : NavSatFix (from sensors_msgs)
		ROS message containing the GPS sensor measurements for the position of the vehicle in gps coordinates.
	*****/
	double gps_aux[3][1] = {{msg->latitude*M_PI/180}, {msg->longitude*M_PI/180}, {msg->altitude}};
	memcpy(sen.gps.pos, gps_aux, sizeof(sen.gps.pos));
}





void TELEMETRY::update_baro_pressure(const sensor_msgs::FluidPressure::ConstPtr& msg){
	/*****
	Updates the variable that stores the static pressure measured by the barometer.

	Parameters
	----------
	msg : FluidPressure (from sensors_msgs)
		ROS message containing the static pressure measurements provided by the barometer.
	*****/
	sen.baro.pressure = msg->fluid_pressure;
}





void TELEMETRY::update_baro_temperature(const sensor_msgs::Temperature::ConstPtr& msg){
	/*****
	Updates the variable that stores the temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.

	Parameters
	----------
	msg : Temperature (from sensors_msgs)
		ROS message containing the temperature measurements provided by the thermometer integrated in the barometer.
	*****/
	sen.baro.temperature = msg->temperature;
}





void TELEMETRY::update_baro_altitude(const mavros_msgs::Altitude::ConstPtr& msg){
	/*****
	Updates the variable that stores the altitude of the vehicle, above mean sea level, computed through the barometric atmospheric pressure and the temperature.

	Parameters
	----------
	msg : Altitude (from mavros_msgs)
		ROS message containing the altitude values computed through the barometric atmospheric pressure and the temperature.
	*****/
	sen.baro.alt = msg->monotonic;
}





void TELEMETRY::update_relative_positions(const std_msgs::Float64MultiArray::ConstPtr& msg){
	/*****
	Updates the variable that stores the relative position of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative position sensor.

	Parameters
	----------
	msg : Float64MultiArray (from std_msgs) 
		ROS message containing a list of the relative position of each of the n neighbour vehicles, in local NED coordinates.
	*****/
	ROS_list_to_array(msg, sen.emu.rel_pos);
}





void TELEMETRY::update_relative_velocities(const std_msgs::Float64MultiArray::ConstPtr& msg){
	/*****
	Updates the variable that stores the relative velocity of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative velocity sensor.

	Parameters
	----------
	msg : Float64MultiArray (from std_msgs) 
		ROS message containing a list of the relative velocity of each of the n neighbour vehicles, in local NED coordinates.
	*****/
	ROS_list_to_array(msg, sen.emu.rel_vel);
}





void TELEMETRY::update_actuator(const mavros_msgs::ActuatorControl::ConstPtr& msg){
	/*****
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.

		Parameters
		----------
		msg : ActuatorControl (from mavros_msgs) 
			ROS message containing the normalized values applied to the mixer and/or motors and servos of the vehicle.
	*****/

	act.group = msg->group_mix;

	double act_aux[8] = {msg->controls[0], msg->controls[1], msg->controls[2], msg->controls[3], msg->controls[4], msg->controls[5], msg->controls[6], msg->controls[7]};
	memcpy(act.output, act_aux, sizeof(act.output));
}





void TELEMETRY::update_status(const mavros_msgs::State::ConstPtr& msg){
	/*****
	Updates the variables that store the current flight mode of the PX4 autopilot, the system status, and the armed state of the vehicle. 

	Parameters
	----------
	msg : State (from mavros_msgs) 
		ROS message containing general information about the status of the drone and of the PX4 autopilot.
	*****/
	info.flight_mode = msg->mode;
	info.is_connected = msg->connected;
	info.is_armed = msg->armed;
	
}





void TELEMETRY::update_landed(const mavros_msgs::ExtendedState::ConstPtr& msg){
	/*****
	Updates the variable that stores the landed state of the drone.

	Parameters
	----------
	msg : ExtendedState (from mavros_msgs) 
		ROS message containing information about the landed state of the drone.
	*****/
	info.is_landed = msg->landed_state == 1;
}





void TELEMETRY::update_battery(const sensor_msgs::BatteryState::ConstPtr& msg){
	/*****
	Updates the variable that stores the remaining battery percentage. 

	Parameters
	----------
	msg : BatteryState (from sensor_msgs) 
		ROS message containing information about the battery of the drone.
	*****/
	info.battery = msg->percentage*100;
}





void enu_to_ned(double v_enu[3][1], double v_ned[3][1]){
	/*****
	Converts a 3x1 physical variable from ENU coordinates to NED coordinates.

	Parameters
	----------
	v_enu : array of doubles with dimensions 3x1
		3x1 physical variable in ENU coordinates.

	v_ned : array of doubles with dimensions 3x1
		Array that will store the 3x1 physical variable in NED coordinates.
	*****/
	double v_aux[3][1] = {{v_enu[1][0]},{v_enu[0][0]},{-v_enu[2][0]}};
	memcpy(v_ned, v_aux, sizeof(v_aux));
}





void ROS_quaternion_to_SI_quaternion(double q_ros[4], double q_si[4][1]){
	/*****
	Converts a ROS quaternion in ENU coordinates to a SI quaternion in NED coordinates.

	Parameters
	----------
	q_ros : array of doubles of size 4
		Quaternion in the ROS format and ENU coordinates.

	q_si : array of doubles with dimensions 4x1
		Variable that will store the quaternion in the SI format and NED coordinates.
	*****/
	double q_aux[4][1] = {{q_ros[3]},{q_ros[1]},{q_ros[0]},{-q_ros[2]}};
	memcpy(q_si, q_aux, sizeof(q_aux));
}





void ROS_list_to_array(const std_msgs::Float64MultiArray::ConstPtr& msg, double rel_param[100][3][1]){
	/*****
	Converts a one dimensional ROS message with 3*n elements into an array with dimension nx3x1.

	Parameters
	----------
	msg : Float64MultiArray (from std_msgs) 
		One dimensional ROS message with 3*n elements containing a list of the relative position or velocity of each of the n neighbour vehicles, in NED coordinates.

	rel_param : array of doubles with dimension nx3x1
		Relative position or velocity of each of the n neighbour vehicles, in NED coordinates.
	*****/
	double p_aux[100][3][1];

	for(int i=0; i<100; i++){
		try {p_aux[i][0][0]=msg->data[i*3];  p_aux[i][1][0]=msg->data[i*3+1]; p_aux[i][2][0]=msg->data[i*3+2];}
		catch (...) {break;} 
	}
	memcpy(rel_param, p_aux, sizeof(p_aux));
}

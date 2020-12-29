#ifndef __UAV_H__
#define __UAV_H__





// Import C++ libraries.
#include <string>
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>





// Include ROS messages.
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>





// Introduction of Namespaces.
using namespace std;





class DRONE_INFO {
	/*****
	Class used to store physical properties and the flight status of the drone. All variables are in SI units.

	Attributes
	----------
	drone_ns : str
		ROS namespace where the data from the PX4 autopilot and the MOCAP system is encapsulated.
	mass : double
		Mass of the drone.
	radius : double
		Radius of the drone.
	height : double
		Height of the drone.
	num_rotors : int
		Number of rotors of the drone.
	thrust_curve : str
		Thrust curve of the drone.
	flight_mode : str
		Current flight mode of the drone. The list of flight modes is available at http://wiki.ros.org/mavros/CustomModes.
	is_connected : bool
		States if the system is connected to the PX4 autopilot. 
	is_armed : bool
		Stores the armed state of the vehicle. If True, the drone is armed.
	is_landed : bool
		Stores the landed state of the vehicle. If True, the drone is landed.
	battery : double
		Remaining battery percentage.
	*****/
	public:
		string drone_ns;
		double mass;
		double radius;
		double height;
		int num_rotors;
		string thrust_curve;
		string flight_mode;
		bool is_connected;
		bool is_armed;
		bool is_landed;
		double battery;
};





class ACTUATORS {
	/*****
	Class used to store the current values applied to the mixer and/or actuators (motors and control devices) of the drone.
	More information available at https://dev.px4.io/v1.9.0/en/concept/mixing.html.

	Attributes
	----------
	group : int
		States the group of the active motors and servos of the drone.
	output : array of doubles of size 8
		Stores the normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
	*****/
	public:
		int group;
		double output[8];
};





class EMULATED {
	/*****
	Class used to store the raw measurements provided by emulated sensors.
	
	Attributes
	----------
	rel_pos : array of doubles with dimension nx3x1
		Relative position of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative position sensor.
	rel_vel : array of doubles with dimension nx3x1
		Relative velocity of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative velocity sensor.
	*****/
	public:
		double rel_pos[100][3][1];
		double rel_vel[100][3][1];
};





class BAROMETER {
	/*****
	Class used to store the raw measurements provided by the barometer. All variables are in SI units.

	Attributes
	----------
	pressure : double
		Static pressure measured by the barometer.
	temperature : double
		Temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.
	alt : double
		Altitude of the vehicle, above mean sea level, computed through the barometric atmospheric pressure and the temperature.
	*****/
	public:
		double pressure;
		double temperature;
		double alt;
};





class GPS {
	/*****
	Class used to store the raw measurements provided by the GPS sensor. All variables are in SI units.

	Attributes
	----------
	pos : array of doubles with dimension 3x1
		Position of the vehicle, in gps coordinates, provided by the GPS sensor.
	*****/
	public:
		double pos[3][1];
};





class MOCAP {
	/*****
	Class used to store the position and attitude of the drone provided by the motion capture system. All variables are in SI units.

	Attributes
	----------
	pos : array of doubles with dimension 3x1
		Position of the vehicle, in local NED coordinates, provided by the motion capture system.
	att_q : array of doubles with dimension 4x1
		Attitude of the vehicle, expressed in quaternions, provided by the motion capture system.
	att_euler : array of doubles with dimension 3x1
		Attitude of the drone, expressed in Euler angles, provided by the motion capture system.
	*****/
	public:
		double pos[3][1];
		double att_q[4][1];
		double att_euler[3][1];
};





class IMU {
	/*****
	Class used to store the raw measurements provided by the IMU. All variables are in SI units.

	Attributes
	----------
	acc_body : array of doubles with dimension 3x1
		Linear acceleration of the vehicle, in body NED coordinates, measured by the IMU.
	ang_vel : array of doubles with dimension 3x1
		Angular velocity of the drone measured by the IMU.
	mag : array of doubles with dimension 3x1
		Magnetic field vector, in body NED coordinates, measured by the IMU. Expressed in Teslas.
	*****/
	public:
		double acc_body[3][1];
		double ang_vel[3][1];
		double mag[3][1];
};





class SENSORS {
	/*****
	Class used to store the raw sensors measurements and the MOCAP pose of the drone.

	Attributes
	----------
	imu : object of the IMU class
		Stores the raw measurements provided by the IMU.
	mocap : object of the MOCAP class
		Stores the position and attitude of the drone provided by the motion capture system.
	gps : object of the GPS class
		Stores the raw measurements provided by the GPS sensor.
	baro : object of the BAROMETER class
		Stores the raw measurements provided by the barometer.
	emu : object of the EMULATOR class
		Stores the raw measurements provided by emulated sensors.
	*****/
	public:
		IMU imu;
		MOCAP mocap;
		GPS gps;
		BAROMETER baro;
		EMULATED emu;
};





class EKF {
	/*****
	Class used to store the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot. All variables are in SI units.

	Attributes
	----------
	pos : array of doubles with dimension 3x1
		Position of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	vel : array of doubles with dimension 3x1
		Linear velocity of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	vel_body : array of doubles with dimension 3x1
		Linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	att_q : array of doubles with dimension 4x1
		Attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.
	att_euler : array of doubles with dimension 3x1
		Attitude of the drone, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.
	ang_vel : array of doubles with dimension 3x1
		Angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	*****/
	public:
		double pos[3][1];
		double vel[3][1];
		double vel_body[3][1];
		double att_q[4][1];
		double att_euler[3][1];
		double ang_vel[3][1];
};





class UAV_AUX {
	/*****
	Auxiliary class.

	Attributes
	----------
	ekf : object of the EKF class
		Stores the state of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	sen : object of the SENSORS class
		Stores the raw sensor measurements and the MOCAP pose of the drone.
	act : object of the ACTUATORS class
		Stores the current values applied to the mixers and/or actuators (motors and control devices) of the vehicle.
	info : object of the DRONE_INFO class
		Stores physical properties and the flight status of the vehicle.
	*****/
	public:
		EKF ekf;
		SENSORS sen;
		ACTUATORS act;
		DRONE_INFO info;
		ros::NodeHandle nh;
};





class OFFBOARD: public UAV_AUX {
	/*****
	Stores the methods that send offboard commands and offboard control references to the PX4 autopilot of the vehicle.

	Methods
	-------
	arm_drone()
		Arms the drone, if it is not already armed.
	start_offboard_mode()
		Changes the flight mode of the PX4 autopilot of the drone to offboard.
	start_offboard_mission()
		Makes the vehicle ready for an offboard experiment by arming it and by changing the flight mode of its PX4 autopilot to offboard.
	set_pos_yaw(double pos[3][1], double yaw, double time)
		Offboard method that sends position and yaw references to the PX4 autopilot of the the drone.
	set_vel_yaw(double vel[3][1], double yaw, double freq)
		Offboard method that sends velocity and yaw references to the PX4 autopilot of the the vehicle.
	set_vel_body_yaw_rate(double vel_body[3][1], double yaw_rate, double freq)
		Offboard method that sends velocity_body and yaw_rate references to the PX4 autopilot of the the drone.
	set_att_thrust(double att_euler[3][1], double att_q[4][1], string att_type, double thrust, double freq)
		Offboard method that sends attitude and thrust references to the PX4 autopilot of the the vehicle.
	set_ang_vel_thrust(double ang_vel[3][1], double thrust, double freq)
		Offboard method that sends angular velocity and thrust references to the PX4 autopilot of the the drone.
	set_act(int group, double output[8], double freq)
		Offboard method that sets the values of the mixers and/or actuators of the vehicle.
	disarm_drone()
		Disarms the vehicle, if it is not already disarmed.
	auto_land()
		Lands the drone, changing its flight mode to auto-land.
	*****/
	public:
		ros::Publisher set_pos_pub, set_vel_pub, set_att_pub, set_ang_vel_pub, set_act_pub;
		void	arm_drone();
		void start_offboard_mode();
		void start_offboard_mission();
		void set_pos_yaw(double pos[3][1], double yaw, double time);
		void set_vel_yaw(double vel[3][1], double yaw, double freq);
		void set_vel_body_yaw_rate(double vel_body[3][1], double yaw_rate, double freq);
		void set_att_thrust(double att_euler[3][1], double att_q[4][1], string att_type, double thrust, double freq);
		void set_ang_vel_thrust(double ang_vel[3][1], double thrust, double freq);
		void set_act(int group, double output[8], double freq);
		void disarm_drone();
		void auto_land();
};





class TELEMETRY: public OFFBOARD {
	/*****
	Stores the methods responsible for keeping the variables of the UAV class up to date.

	Methods
	-------
	init_telemetry()
		Manages topic subscriptions.
	update_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
		Updates the variable that stores the position of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
		Updates the variable that stores the linear velocity of the vehicle, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity_body(const geometry_msgs::TwistStamped::ConstPtr& msg)
		Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude(const geometry_msgs::PoseStamped::ConstPtr& msg)
		Updates the variables that store the attitude of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_angular_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_imu(const sensor_msgs::Imu::ConstPtr& msg)
		Updates the variables that store the linear acceleration and the angular velocity of the drone measured by the IMU.
	update_mag(const sensor_msgs::MagneticField::ConstPtr& msg)
		Updates the variable that stores the magnetic field vector, in body NED coordinates, measured by the IMU.
	update_mocap(const geometry_msgs::PoseStamped::ConstPtr& msg)
		Updates the variables that store the position and attitude of the drone provided by the motion capture system.
	update_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
		Updates the variable that stores the raw measurements provided by the GPS sensor.
	update_baro_pressure(const sensor_msgs::FluidPressure::ConstPtr& msg)
		Updates the variable that stores the static pressure measured by the barometer.
	update_baro_temperature(const sensor_msgs::Temperature::ConstPtr& msg)
		Updates the variable that stores the temperature, in degrees Kelvin, measured by the thermometer integrated in the barometer.
	update_baro_altitude(const mavros_msgs::Altitude::ConstPtr& msg)
		Updates the variable that stores the altitude of the vehicle, above mean sea level, computed through the barometric atmospheric pressure and the temperature.
	update_relative_positions(const std_msgs::Float64MultiArray::ConstPtr& msg)
		Updates the variable that stores the relative position of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative position sensor.
	update_relative_velocities(const std_msgs::Float64MultiArray::ConstPtr& msg)
		Updates the variable that stores the relative velocity of each of the n neighbour vehicles, in local NED coordinates, provided by the emulated relative velocity sensor.
	update_actuator(const mavros_msgs::ActuatorControl::ConstPtr& msg)
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
	update_status(const mavros_msgs::State::ConstPtr& msg)
		Updates the variables that store the current flight mode of the PX4 autopilot, the system status, and the armed state of the vehicle. 
	update_landed(const mavros_msgs::ExtendedState::ConstPtr& msg)
		Updates the variable that stores the landed state of the drone.
	update_battery(const sensor_msgs::BatteryState::ConstPtr& msg)
		Updates the variable that stores the remaining battery percentage. 
	*****/
	public:
		void init_telemetry();
		void update_position(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void update_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
		void update_velocity_body(const geometry_msgs::TwistStamped::ConstPtr& msg);
		void update_attitude(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void update_angular_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg);
		void update_imu(const sensor_msgs::Imu::ConstPtr& msg);
		void update_mag(const sensor_msgs::MagneticField::ConstPtr& msg);
		void update_mocap(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void update_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
		void update_baro_pressure(const sensor_msgs::FluidPressure::ConstPtr& msg);
		void update_baro_temperature(const sensor_msgs::Temperature::ConstPtr& msg);
		void update_baro_altitude(const mavros_msgs::Altitude::ConstPtr& msg);
		void update_relative_positions(const std_msgs::Float64MultiArray::ConstPtr& msg);
		void update_relative_velocities(const std_msgs::Float64MultiArray::ConstPtr& msg);
		void update_actuator(const mavros_msgs::ActuatorControl::ConstPtr& msg);
		void update_status(const mavros_msgs::State::ConstPtr& msg);
		void update_landed(const mavros_msgs::ExtendedState::ConstPtr& msg);
		void update_battery(const sensor_msgs::BatteryState::ConstPtr& msg);
};





class UAV: public TELEMETRY {
	/*****
	Class used to represent an UAV.

	Methods
	-------
	UAV(string drone_ns, string mass, string radius, string height, string num_rotors, string thrust_curve)
		Constructor of the UAV class. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
	*****/
	public:
		UAV(string drone_ns, string mass, string radius, string height, string num_rotors, string thrust_curve);
};


#endif

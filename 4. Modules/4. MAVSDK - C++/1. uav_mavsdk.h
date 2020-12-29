#ifndef __UAV_H__
#define __UAV_H__





// Import C++ libraries
#include <string>
#include <thread>
#include <math.h>
#include <chrono>
#include <cstring>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>





// Introduction of Namespaces
using namespace std;
using namespace mavsdk;
using namespace std::chrono;
using namespace std::this_thread;
using namespace std::placeholders;





class DRONE_INFO {
	/*****
	Class used to store physical properties and the flight status of the drone. All variables are in SI units.

	Attributes
	----------
	drone_udp : str
		UDP address of the PX4 autopilot.
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
		string drone_udp;
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
		Mavsdk dc;
    		System& system = dc.system();

		EKF ekf;
		ACTUATORS act;
		DRONE_INFO info;
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
	disarm_drone()
		Disarms the vehicle, if it is not already disarmed.
	auto_land()
		Lands the drone, changing its flight mode to auto-land.
	*****/
	public:
		shared_ptr<mavsdk::Action> action = std::make_shared<Action>(system);
		shared_ptr<mavsdk::Offboard> offboard = std::make_shared<Offboard>(system);

		void	arm_drone();
		void start_offboard_mode();
		void start_offboard_mission();
		void set_pos_yaw(double pos[3][1], double yaw, double time);
		void set_vel_yaw(double vel[3][1], double yaw, double freq);
		void set_vel_body_yaw_rate(double vel_body[3][1], double yaw_rate, double freq);
		void set_att_thrust(double att_euler[3][1], double att_q[4][1], string att_type, double thrust, double freq);
		void set_ang_vel_thrust(double ang_vel[3][1], double thrust, double freq);
		void disarm_drone();
		void auto_land();
};





class TELEMETRY: public OFFBOARD {
	/*****
	Stores the methods responsible for keeping the variables of the UAV class up to date.

	Methods
	-------
	init_telemetry()
		Defines the callback methods responsible for updating each variable of the UAV class.
	update_position_and_velocity(Telemetry::PositionVelocityNED msg)
		Updates the variables that store the position and linear velocity of the drone, in local NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_velocity_body(Telemetry::Odometry msg)
		Updates the variable that stores the linear velocity of the vehicle, in body NED coordinates, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude_quaternion(Telemetry::Quaternion msg)
		Updates the variable that stores the attitude of the vehicle, expressed in quaternions, provided by the extended Kalman filter of the PX4 autopilot.
	update_attitude_euler(Telemetry::EulerAngle msg)
		Updates the variable that stores the attitude of the vehicle, expressed in Euler angles, provided by the extended Kalman filter of the PX4 autopilot.
	update_angular_velocity(Telemetry::AngularVelocityBody msg)
		Updates the variable that stores the angular velocity of the vehicle provided by the extended Kalman filter of the PX4 autopilot.
	update_actuator(Telemetry::ActuatorControlTarget msg)
		Updates the variables that store the group and the current normalized values (0 to 1 or -1 to 1) applied to the mixer and/or motors and servos of the vehicle.
	update_flight_mode(Telemetry::FlightMode msg)
		Updates the variable that stores the current flight mode of the PX4 autopilot of the vehicle. 
	update_status(bool msg)
		Updates the variable that stores the armed state of the vehicle. 
	update_landed(bool msg)
		Updates the variable that stores the landed state of the drone.
	update_battery(Telemetry::Battery msg)
		Updates the variable that stores the remaining battery percentage. 
	*****/
	public:
		shared_ptr<mavsdk::Telemetry> telemetry = make_shared<Telemetry>(system);

		void init_telemetry();
		void update_position_and_velocity(Telemetry::PositionVelocityNED msg);
		void update_velocity_body(Telemetry::Odometry msg);
		void update_attitude_quaternion(Telemetry::Quaternion msg);
		void update_attitude_euler(Telemetry::EulerAngle msg);
		void update_angular_velocity(Telemetry::AngularVelocityBody msg);
		void update_actuator(Telemetry::ActuatorControlTarget msg);
		void update_flight_mode(Telemetry::FlightMode msg);
		void update_status(bool msg);
		void update_landed(bool msg);
		void update_battery(Telemetry::Battery msg);
};





class UAV: public TELEMETRY {
	/*****
	Class used to represent an UAV.

	Methods
	-------
	UAV(string drone_udp, string mass, string radius, string height, string num_rotors, string thrust_curve);
		Constructor of the UAV class. Starts a background thread responsible for keeping all the variables of the UAV class up to date. 
	*****/
	public:
		UAV(string drone_udp, string mass, string radius, string height, string num_rotors, string thrust_curve);
};


#endif

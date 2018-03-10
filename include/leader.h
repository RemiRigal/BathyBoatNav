#ifndef _LEADER_H
#define _LEADER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"

// Quaternions
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

// FSM states
#include "../include/state.h" 

// Messages
#include "BathyBoatNav/robot_state.h"
#include "BathyBoatNav/robot_target.h"

// Services
#include "std_srvs/SetBool.h"
#include "BathyBoatNav/message.h"
#include "BathyBoatNav/new_state.h"
#include "BathyBoatNav/next_goal.h"
#include "BathyBoatNav/gps_conversion.h"

class Leader{

	public: 
		Leader();
	    virtual ~Leader();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;
		
		// Simulation
		bool isSimulation;

		// Robot state variables
		State state;
		double x[3];
		double converted_x[3];
		tf::Quaternion q;
		double linear_speed[3];
		double angular_speed[3];
		int gps_status;
		
		std::vector<double> batt;

		// Robot target variables
		bool isLine;
		int id;
		double x_target[3];
		tf::Quaternion q_target;
		double x_target_appoint[3];
		tf::Quaternion q_target_appoint;
		double linear_speed_target[3];
		double angular_speed_target[3];

		// Raw robot state msg
		ros::Publisher robot_state_raw_pub;
		void updateRobotStateMsg();

		// Converted robot state msg
		ros::Publisher robot_state_converted_pub;
		void updateRobotStateConvertedMsg();

		// Target
		ros::Publisher robot_target_pub;
		void updateRobotTargetMsg();

		// Parsing TCP server inputs
		ros::ServiceServer serverInputs;
		bool parseCommand(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res);

		// Conversion with proj4
		ros::ServiceClient convert_coords_client;

		// Changing state
		BathyBoatNav::new_state new_state_msg;
		ros::ServiceClient changeStateSrv;
		bool setState(std::string state);

		// New mission
		double accept_dist;
		BathyBoatNav::message mission_path_msg;
		ros::ServiceClient mision_path_client;
		bool changeMission(std::string path);
		bool missionReady(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

		// Validating targets
		BathyBoatNav::next_goal next_goal_msg;
		ros::ServiceClient next_goal_client;
		void checkIfTargetValidated();
		void askForNewWaypoints();
		
		//PID
		double k_P, k_I, k_D;
		void changePID(double P, double I);

		// Speed
		void changeSpeed(double speed);

		// Sensors		
		void gatherData();

		// Evolution of status if simulation
		ros::Subscriber robot_state_converted_evolved_sub;
		ros::Subscriber robot_state_raw_evolved_sub;
		void updateRobotStateConvertedEvolved(const BathyBoatNav::robot_state::ConstPtr& msg);
		void updateRobotStateRawEvolved(const BathyBoatNav::robot_state::ConstPtr& msg);

};

#endif
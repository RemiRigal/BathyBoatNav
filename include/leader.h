#ifndef _LEADER_H
#define _LEADER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "ros/ros.h"

// Quaternions
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include <boost/algorithm/string.hpp> // Parsing strings

#include "../include/state.h" // FSM states

// Messages
#include "BathyBoatNav/robot_state.h"
#include "BathyBoatNav/robot_target.h"

// Services
#include "std_srvs/SetBool.h"
#include "BathyBoatNav/message.h"
#include "BathyBoatNav/new_state.h"
#include "BathyBoatNav/next_goal.h"

class Leader{

	public: 
		Leader();
	    virtual ~Leader();
		void RunContinuously();
    
	private: 
		ros::NodeHandle* Handle;
		
		// Robot state variables
		State state;
		double x[3];
		tf::Quaternion q;
		double linear_speed[3];
		double angular_speed[3];
		int gps_status;
		double k_P, k_I, k_D; // PID factors

		// Robot target variables
		bool isLine;
		int id;
		double x_target[3];
		tf::Quaternion q_target;
		double x_target_appoint[3];
		tf::Quaternion q_target_appoint;
		double linear_speed_target[3];
		double angular_speed_target[3];

		// Robot state msg
		BathyBoatNav::robot_state robot_state_msg;
		ros::Publisher robot_state_pub;
		void updateRobotStateMsg();

		// Target
		BathyBoatNav::robot_target robot_target_msg;
		ros::Publisher robot_target_pub;
		void updateRobotTargetMsg();

		// Parsing TCP server inputs
		ros::ServiceServer serverInputs;

		// Changing state
		BathyBoatNav::new_state new_state_msg;
		ros::ServiceClient changeStateSrv;

		// New mission
		double accept_dist;
		BathyBoatNav::message mission_path_msg;
		ros::ServiceClient mision_path_client;
		bool missionReady(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

		// Validating targets
		BathyBoatNav::next_goal next_goal_msg;
		ros::ServiceClient next_goal_client;
		void checkIfTargetValidated();
		void askForNewWaypoints();

		bool setState(State state);

		bool changeMission(std::string path);

		void changePID(double P, double I);

		void changeSpeed(double speed);

		
		bool parseCommand(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res);
		
		void gatherData();

};

#endif
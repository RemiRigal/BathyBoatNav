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
#include "BathyBoatNav/message.h"
#include "BathyBoatNav/new_state.h"

class Leader{

	public: 
		Leader();
	    virtual ~Leader();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;
		
		// Robot state variables
		State state;
		double x[3];
		tf::Quaternion q;
		double linear_speed[3];
		double angular_speed[3];
		int gps_status;
		double instructions[];

		// Robot state msg
		BathyBoatNav::robot_state robot_state_msg;
		ros::Publisher robot_state_pub;
		bool updateRobotStateMsg();

		// Target
		BathyBoatNav::robot_target robot_target_msg;
		ros::Publisher robot_target_pub;
		bool updateRobotTargetMsg();

		// Parsing TCP server inputs
		ros::ServiceServer serverInputs;

		// Changing state
		BathyBoatNav::new_state new_state_msg;
		ros::ServiceClient changeStateSrv;

		bool setState(State state);

		void changeMission(std::string path);

		void changePID(double P, double I);

		void changeSpeed(double speed);

		bool parseCommand(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res);
		bool checkIfTargetValidated();
		void gatherData();

};

#endif
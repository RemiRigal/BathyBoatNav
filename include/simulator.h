#ifndef _SIMULATOR_H
#define _SIMULATOR_H

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
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include "BathyBoatNav/robot_state.h"
#include "BathyBoatNav/robot_target.h"

// Services
#include "BathyBoatNav/gps_conversion.h"

class Simulator{

	public: 
		Simulator();
	    virtual ~Simulator();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;

		// Robot state variables
		State state;
		double x[3];
		double converted_x[3];
		double roll, pitch, yaw;
		double linear_speed[3];
		double angular_speed[3];
		tf::Quaternion q;

		// Robot state
		ros::Subscriber robot_state_sub;
		void updateRobotState(const BathyBoatNav::robot_state::ConstPtr& msg);

		// Command
		ros::Subscriber command_sub;
		double u_yaw, speed_bar;
		void updateCommand(const geometry_msgs::Twist::ConstPtr& msg);

		// Simulation
		double speed;
		void evolution();

		// Simulated state
		ros::Publisher robot_state_converted_evolved_pub;
		ros::Publisher robot_state_raw_evolved_pub;
		void updateRobotStateConvertedEvolvedMsg();
		void updateRobotStateRawEvolvedMsg();

		// Conversion with proj4
		ros::ServiceClient convert_coords_client;
};

#endif
#ifndef _REGULATOR_H
#define _REGULATOR_H

#include <iostream>
#include <string>
#include <cmath>

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "BathyBoatNav/next_goal.h"
#include "BathyBoatNav/new_state.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "../include/state.h"

class Regulator{

	public: 
		Regulator();
	    virtual ~Regulator();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;
		
		void computeDistance();

		void posCallback(const geometry_msgs::Twist::ConstPtr& msg);

#endif
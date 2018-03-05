#ifndef _TARGET_H
#define _TARGET_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"

#include "../include/state.h"

class Target{

	public: 
		Target();
	    virtual ~Target();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;

		ros::Publisher state_pub;
		std_msgs::Int16 state_msg;
		
	    ros::ServiceClient next_target_client;
	    BathyBoatNav::next_goal next_target_msg;

		ros::ServiceServer changeStateSrv;
		
		ros::ServiceClient pololuLeader;

		void computeDistance();

};

#endif

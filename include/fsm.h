#ifndef _FSM_H
#define _FSM_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Int16.h"
#include "BathyBoatNav/message.h"
#include "BathyBoatNav/new_state.h"

#include "../include/state.h"

class FSM{

	public: 
		FSM();
	    virtual ~FSM();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;

		ros::Publisher state_pub;
		std_msgs::Int16 state_msg;
		
		ros::ServiceServer changeStateSrv;
		
		ros::ServiceClient pololuLeader;
		ros::ServiceClient regulLeader;
		ros::ServiceClient missionReady;

		BathyBoatNav::new_state new_state_msg;
		std_srvs::Trigger mission_ready_msg;
		
		State state;

		bool changeState(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res);
		void setState(State newState);
		bool advertChangeState();
};

#endif

#ifndef _FSM_H
#define _FSM_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "BathyBoatNav/changeState.h"

enum State{
    INIT 		= 0,
    RUNNING 	= 1,
    IDLE 		= 2,
    RTL 		= 3,
    STOP 		= 4,
    EMERGENCY 	= 5
};

class FSM{

	public: 
		FSM();
	    virtual ~FSM();
		void RunContinuously();
    
	private: 
		ros::NodeHandle Handle;
		ros::ServiceServer changeStateSrv;
		State state;

		bool changeState(BathyBoatNav::changeState::Request &req, BathyBoatNav::changeState::Response &res);
		void setState(State newState);
};

#endif

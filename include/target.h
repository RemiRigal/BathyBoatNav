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
};

#endif

#ifndef _MISSION_INTERPRETER_HH
#define _MISSION_INTERPRETER_HH

#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <list>
#include <stdio.h>
#include "ros/ros.h"
#include "BathyBoatNav/next_goal.h"
#include "std_srvs/Empty.h"
#include "string.h"


class MissionInterpreter{

    public: 
        MissionInterpreter();
	    virtual ~MissionInterpreter();
        void Prepare();
        void RunContinuously();
    
    private: 
        void ReadFile();
        void FillArrayPoints();
        std::list<BathyBoatNav::next_goal::Response> FillLongLat(int i, std::vector<std::string> array, BathyBoatNav::next_goal::Response goalTemp);
        std::list<BathyBoatNav::next_goal::Response> FindRadiales(int i, BathyBoatNav::next_goal::Response goalTemp);
        bool sendPointService(BathyBoatNav::next_goal::Request &req, BathyBoatNav::next_goal::Response &res);
        ros::NodeHandle Handle; 
        ros::ServiceServer SendPointSrv;
        BathyBoatNav::next_goal::Response goalPoint;
        std::vector<std::string> arrayFile;
        std::list<BathyBoatNav::next_goal::Response> goalPointsList;
		std::list<std::list<BathyBoatNav::next_goal::Response> > missionList;
        std::ifstream jsonFile;
    	int nbrMissions;
};

#endif

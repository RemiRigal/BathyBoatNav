#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <list>
#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_srvs/Empty.h"
#include "string.h"

class MissionInterpreter{

    public: 
        void Prepare();
        void RunContinuously();
        bool readyCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    
    private: 
        void ReadFile();
        void FillArrayPoints();
        ros::NodeHandle Handle; 
        ros::ServiceServer SendPointSrv;
        ros::Publisher wayPointPub;
        sensor_msgs::NavSatFix wayPoint;
        float longitude;
        float latitude;
        std::string typeMission;
        std::vector<std::string> arrayFile;
        std::list<sensor_msgs::NavSatFix> WayPointsList;
		std::list<std::list<sensor_msgs::NavSatFix> > missionList;
        std::ifstream jsonFile;
    	int nbrMissions;
};

void MissionInterpreter::Prepare() {
    nbrMissions = 0;
    ReadFile();
    FillArrayPoints();
    SendPointSrv = Handle.advertiseService("/msg_send_waypoint", &MissionInterpreter::readyCallback, this);    
    wayPointPub = Handle.advertise<sensor_msgs::NavSatFix>("next_waypoint", 10);
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void MissionInterpreter::ReadFile() {
    std::string value;
    jsonFile.open("/home/noelie/Documents/ProjetGuerledan/Noelie/mission.json", std::ios::app);
    if (jsonFile.is_open()){
        while(jsonFile >> value){
            arrayFile.push_back(value);
            //std::cout << "value" << value << std::endl;
            //ROS_INFO("value :", value);
        }
    jsonFile.close();
    }
    else {
        ROS_INFO("Unable to open json mission file.");
    }
}

void MissionInterpreter::FillArrayPoints() {
    for (int i=0; i < arrayFile.size(); i++){
        if (arrayFile[i] == "\"type\":"){
            if (arrayFile[i+1] == "\"WayPoint\","){
                typeMission = "WayPoint";
				sensor_msgs::NavSatFix wayPointTemp;
				for (int j=i+1; j < arrayFile.size(); j++){
					if (arrayFile[j] == "\"latitude\":"){
						wayPointTemp.latitude = ::atof(arrayFile[j+1].c_str());
					}
					if (arrayFile[j] == "\"longitude\":"){
						wayPointTemp.longitude = ::atof(arrayFile[j+1].c_str());
						WayPointsList.push_back(wayPointTemp);
						//ROS_INFO("wayPoint : lat : %f ", wayPointTemp.latitude);
						//ROS_INFO("wayPoint : long : %f" , wayPointTemp.longitude);
					}
					if (arrayFile[j] == "\"type\":"){
						break;
					}
				}
				missionList.push_back(WayPointsList);
				WayPointsList.clear();
            }else {
            	typeMission = "Radiales";
            }
        nbrMissions ++;
        }
    }
    
}


bool MissionInterpreter::readyCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    if (nbrMissions > 0){
		wayPoint = missionList.front().front();
		missionList.front().pop_front();
		wayPointPub.publish(wayPoint);
		if (missionList.front().empty()){
			missionList.pop_front();
			nbrMissions --;
			ROS_INFO("Changement de mission. \nNombre de missions : %d", nbrMissions);
		}	
		return true;
    }else{
    	return false;
    }

}

void MissionInterpreter::RunContinuously() {    
    ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
    ros::spin();
}


int main(int argc, char** argv){
  ros::init(argc,argv,"mission_interpreter");
  MissionInterpreter interpret;
  interpret.Prepare();
  interpret.RunContinuously();
  return(0);
}

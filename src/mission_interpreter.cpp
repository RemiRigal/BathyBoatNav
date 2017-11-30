#include "../include/mission_interpreter.hh"


MissionInterpreter::MissionInterpreter(){}

MissionInterpreter::~MissionInterpreter(){}


void MissionInterpreter::Prepare() {
    nbrMissions = 0;
    ReadFile();
    FillArrayPoints();
    SendPointSrv = Handle.advertiseService("/next_goal", &MissionInterpreter::sendPointService, this);   
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void MissionInterpreter::ReadFile() {
    std::string value;
    jsonFile.open("/home/noelie/Documents/ProjetGuerledan/Noelie/mission.json", std::ios::app);
    if (jsonFile.is_open()){
        while(jsonFile >> value){
            arrayFile.push_back(value);
        }
    jsonFile.close();
    }
    else {
        ROS_INFO("Unable to open json mission file.");
    }
}

std::list<BathyBoatNav::next_goal::Response> MissionInterpreter::FillLongLat(int i, std::vector<std::string> array, BathyBoatNav::next_goal::Response point) {
	std::list<BathyBoatNav::next_goal::Response> list;
	for (int j=i+1; j < arrayFile.size(); j++){
		if (arrayFile[j] == "\"latitude\":"){
			point.latitude.push_back(::atof(array[j+1].c_str()));
		}
		if (arrayFile[j] == "\"longitude\":"){
			point.longitude.push_back(::atof(array[j+1].c_str()));
			list.push_back(point);
			point.latitude.clear();
			point.longitude.clear();
		}
		if (arrayFile[j] == "\"type\":"){
			break;
		}
	}
	return list;
}

std::list<BathyBoatNav::next_goal::Response> MissionInterpreter::FindRadiales(int i, BathyBoatNav::next_goal::Response goalTemp){
	double angle;
	std::vector<float> coordPoint;
	std::list<BathyBoatNav::next_goal::Response> polyPointlist; 
	std::list<BathyBoatNav::next_goal::Response> list; 
	for (int j=i+1; j < arrayFile.size(); j++){
		if (arrayFile[j] == "\"point\":"){
			coordPoint.push_back(::atof(arrayFile[j+2].c_str()));
			coordPoint.push_back(::atof(arrayFile[j+4].c_str()));
		}
		if (arrayFile[j] == "\"angle\":"){
			angle = ::atof(arrayFile[j+1].c_str());
		}
		if (arrayFile[j] == "\"polygon\":"){
			polyPointlist = FillLongLat(j, arrayFile, goalTemp);
			//list = algoPourFaireRadiales(polyPointList, goalTemp); 
		}
	}
	return list;
}

void MissionInterpreter::FillArrayPoints() {
    for (int i=0; i < arrayFile.size(); i++){
    	BathyBoatNav::next_goal::Response goalTemp;
        if (arrayFile[i] == "\"type\":"){
            if (arrayFile[i+1] == "\"WayPoint\","){
				goalTemp.isRadiale = false;
				//ROS_INFO("Algo pour trouver les waypoints");
				goalPointsList = FillLongLat(i, arrayFile, goalTemp);
            }else {
            	goalTemp.isRadiale = true;
            	goalPointsList = FindRadiales(i, goalTemp);
            	//ROS_INFO("Algo pour trouver les radiales");
            }
        missionList.push_back(goalPointsList);
		goalPointsList.clear();
        nbrMissions ++;
        }
    }
    
}


bool MissionInterpreter::sendPointService(BathyBoatNav::next_goal::Request &req, BathyBoatNav::next_goal::Response &res) {
    if (nbrMissions > 0 && !missionList.front().empty()){
		goalPoint = missionList.front().front();
		missionList.front().pop_front();
		if (missionList.front().empty()){
			missionList.pop_front();
			nbrMissions --;
			ROS_INFO("Changement de mission. \nNombre de missions : %d", nbrMissions);
		}
		goalPoint.nbrMissionNext = nbrMissions;
		res = goalPoint;	
		return true;
    }else{
    	ROS_INFO("Pas de mission ou mission radiales (implémentation en cours) ");
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

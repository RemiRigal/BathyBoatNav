#include "../include/leader.h"

using namespace std;

Leader::Leader()
{
	// Robot state
	robot_state_pub = Handle.advertise<BathyBoatNav::robot_state>("/robot_state", 1000);

	// Robot target
	robot_target_pub = Handle.advertise<BathyBoatNav::robot_state>("/robot_target", 1000);

	// Parsing TCP inputs
	serverInputs = Handle.advertiseService("/TCP_inputs", &Leader::parseCommand, this);

	// Change state of FSM
	changeStateSrv = Handle.serviceClient<BathyBoatNav::new_state>("changeStateSrv");
}

Leader::~Leader()
{}

void Leader::RunContinuously()
{
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

bool Leader::setState(State state)
{
	new_state_msg.request.state = state;
	if (changeStateSrv.call(new_state_msg))
	{
		if(!new_state_msg.response.success)
		{
			ROS_ERROR("FSM failed to change state");
			return false;
		}
	} else{
		ROS_ERROR("Failed to call FSM");
		return false;
	}

	return true;
}

bool Leader::parseCommand(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res)
{
	string msg = req.message;
	vector<string> split_msg;

	bool error = false;

	boost::split(split_msg, msg, boost::is_any_of("|"));

	int sizeVector = split_msg.size();

	if(sizeVector != 0)
	{
		if(split_msg[0] == "MISSION")
		{
			if(sizeVector == 2)
			{
				Leader::changeMission(split_msg[1]);
			} else {
				ROS_INFO("Mission message canvas is \"MISSION|name_of_mission.json\" but received \"%s\"", msg.c_str());
				error = true;
			}
		} else if(split_msg[0] == "FACTOR") {
			if(sizeVector == 3)
			{
				Leader::changePID(atof(split_msg[1].c_str()), atof(split_msg[2].c_str()));
			} else {
				ROS_INFO("PID message canvas is \"FACTOR|k_I|k_P\" but received \"%s\"", msg.c_str());
				error = true;
			}
		} else if(split_msg[0] == "SPEED") {
			if(sizeVector == 3)
			{
				Leader::changeSpeed(atof(split_msg[1].c_str()));
			} else {
				ROS_INFO("Speed message canvas is \"SPEED|coeffSPD\" but received \"%s\"", msg.c_str());
				error = true;
			}
		} else {
			ROS_WARN("Received message %s but parsing give no meaning to it.", msg.c_str());
		}		
	} else {
		error = true;
	}

	return error;
}

void Leader::changeMission(string path)
{
	ROS_INFO("New mission path %s", path.c_str());
}

void Leader::changePID(double P, double I)
{
	ROS_INFO("Change of PID factor to k_P = %lf and k_I = %lf", P, I);
}

void Leader::changeSpeed(double speed)
{
	ROS_INFO("Change of speed factor to %lf", speed);
}

bool Leader::checkIfTargetValidated()
{
	if(false)
	{
		return true;
	}

	return false;
}

bool Leader::updateRobotStateMsg()
{

}

bool Leader::updateRobotTargetMsg()
{

}

void Leader::gatherData()
{

}

int main(int argc, char** argv){
	ros::init(argc,argv,"leader");
	Leader leader = Leader();
	leader.RunContinuously();
	return EXIT_SUCCESS;
}
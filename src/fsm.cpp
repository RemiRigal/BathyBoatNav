#include "../include/fsm.h"

using namespace std;

FSM::FSM()
{
	changeStateSrv = Handle.advertiseService("changeStateSrv", &FSM::changeState, this);
}

FSM::~FSM()
{}

void FSM::RunContinuously()
{
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

bool FSM::changeState(BathyBoatNav::new_state::Request &req, BathyBoatNav::new_state::Response &res)
{
	if(state == EMERGENCY)
	{
		res.success = false;
		return true;
	}

	vector<string> split_msg;

	string msg = req.state;

	boost::split(split_msg, msg, boost::is_any_of(" "));

	res.success = true;

	if(split_msg[0] == "IDLE")
	{
		FSM::setState(IDLE);
	} else if (split_msg[0] == "RESUME" && state == PAUSE) {
		FSM::setState(RUNNING);
	} else if (split_msg[0] == "PAUSE") {
		FSM::setState(PAUSE);
	} else if (split_msg[0] == "RTL") {
		FSM::setState(RTL);
	} else if (split_msg[0] == "STOP") {
		FSM::setState(IDLE);
	} else if (split_msg[0] == "EMERGENCY") {
		FSM::setState(EMERGENCY);
	} else {
		res.success = false;
		return false;
	}

	res.state = state;

	return true;
}

void FSM::setState(State newState)
{
	if (state == newState)
	{
		return;
	}

	state = newState;
	ROS_INFO("Current state -> %d", state);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"fsm");
	FSM fsm = FSM();
	fsm.RunContinuously();
	return EXIT_SUCCESS;
}
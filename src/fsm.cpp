#include "../include/fsm.h"

using namespace std;

FSM::FSM()
{
	state = INIT;
	changeStateSrv = Handle.advertiseService("/changeStateSrv", &FSM::changeState, this);
	ROS_INFO("Constructor done");
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

bool FSM::changeState(BathyBoatNav::changeState::Request &req, BathyBoatNav::changeState::Response &res)
{
	vector<string> split_msg;

	string msg = req.message;

	boost::split(split_msg, msg, boost::is_any_of(" "));

	if(split_msg[0] == "Init")
	{
		FSM::setState(INIT);
	} else if (split_msg[0] == "Run") {
		FSM::setState(RUNNING);
	} else if (split_msg[0] == "Idle") {
		FSM::setState(IDLE);
	} else if (split_msg[0] == "RTL") {
		FSM::setState(RTL);
	} else if 	(split_msg[0] == "Stop") {
		FSM::setState(STOP);
	} else if (split_msg[0] == "Emergency") {
		FSM::setState(EMERGENCY);
	}

	res.success = true;

    ROS_INFO("Message received : %s", msg.c_str());

    return true;
}

void FSM::setState(State newState)
{
	if (state != newState)
	{
		state = newState;
		ROS_INFO("Current state -> %d", state);
	}
}

int main(int argc, char** argv){
	ros::init(argc,argv,"fsm");
	FSM fsm = FSM();
	fsm.RunContinuously();
	return EXIT_SUCCESS;
}
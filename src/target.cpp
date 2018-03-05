#include "../include/target.h"

using namespace std;

FSM::FSM()
{
	FSM::setState(IDLE);

	state_pub = Handle.advertise<std_msgs::Int16>("current_state", 1000);

	changeStateSrv = Handle.advertiseService("changeStateSrv", &FSM::changeState, this);

	pololuLeader 	= Handle.serviceClient<BathyBoatNav::new_state>("pololu_state");
	regulLeader 	= Handle.serviceClient<BathyBoatNav::new_state>("regul_state");
	missionReady 	= Handle.serviceClient<std_srvs::Trigger>("mission_ready");

	while(!FSM::advertChangeState())
	{
		ROS_INFO("Trying to call nodes");
		sleep(1);
	}

	ROS_INFO("Initial state set");
}

FSM::~FSM()
{}

void FSM::RunContinuously()
{
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		state_msg.data = state;
		state_pub.publish(state_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

bool FSM::changeState(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res)
{
	if(state == EMERGENCY)
	{
		res.success = false;
		return true;
	}

	vector<string> split_msg;

	string msg = req.message;

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
	}

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

	FSM::advertChangeState();
}

bool FSM::advertChangeState()
{
	bool pololu = false;
	bool regul = false;

	new_state_msg.request.state = state;
	
	if (pololuLeader.call(new_state_msg))
	{
		pololu = true;
		if(!new_state_msg.response.success)
		{
			ROS_ERROR("Pololu failed to change state");
		}
	} else{
		ROS_ERROR("Failed to call pololu");
	}

	if (regulLeader.call(new_state_msg))
	{
		regul = true;
		if(!new_state_msg.response.success)
		{
			ROS_ERROR("Pololu failed to change state");
		}
	} else{
		ROS_ERROR("Failed to call regulator");
	}

	return pololu && regul;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"fsm");
	FSM fsm = FSM();
	fsm.RunContinuously();
	return EXIT_SUCCESS;
}
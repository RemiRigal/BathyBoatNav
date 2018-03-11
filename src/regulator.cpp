#include "../include/regulator.h"

using namespace std;

Regulator::Regulator()
{
	// Import datas
	Handle.getParam("/isSimu", isSimulation);
	Handle.getParam("/regulation/full_left", full_left);

	// Robot state
	robot_state_sub = Handle.subscribe("/robot_state_converted", 1000, &Regulator::updateRobotState, this);

	// Robot target
	robot_target_sub = Handle.subscribe("/robot_target", 1000, &Regulator::updateRobotTarget, this);

	// Command
	command_pub = Handle.advertise<geometry_msgs::Twist>("/cons_helios", 1000);
	debug_pub 	= Handle.advertise<std_msgs::Float64MultiArray>("debug_helios", 1000);

	// PID
	P, I = 0.0;
}

Regulator::~Regulator()
{}

void Regulator::RunContinuously()
{
	// Loop
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		if(state == RUNNING)
		{
			Regulator::updateCommandMsg();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Regulator::updateRobotState(const BathyBoatNav::robot_state::ConstPtr& msg)
{
	state = State(msg->state);

	x[0] = msg->pose.position.x;
	x[1] = msg->pose.position.y;
	x[2] = msg->pose.position.z;

	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	linear_speed[0] = msg->speed.linear.x;
	linear_speed[1] = msg->speed.linear.y;
	linear_speed[2] = msg->speed.linear.z;
	angular_speed[0] = msg->speed.angular.x;
	angular_speed[1] = msg->speed.angular.y;
	angular_speed[2] = msg->speed.angular.z;

	k_P = msg->pid.k_P;
	k_I = msg->pid.k_I;
	k_D = msg->pid.k_D;
}

void Regulator::updateRobotTarget(const BathyBoatNav::robot_target::ConstPtr& msg)
{	
	isLine = msg->isLine;

	x_target[0] = msg->pose.position.x;
	x_target[1] = msg->pose.position.y;
	x_target[2] = msg->pose.position.z;

	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll_target, pitch_target, yaw_target);

	x_target_appoint[0] = msg->pose_appoint.position.x;
	x_target_appoint[1] = msg->pose_appoint.position.y;
	x_target_appoint[2] = msg->pose_appoint.position.z;

	linear_speed_target[0] = msg->speed.linear.x;
	linear_speed_target[1] = msg->speed.linear.y;
	linear_speed_target[2] = msg->speed.linear.z;
	angular_speed_target[0] = msg->speed.angular.x;
	angular_speed_target[1] = msg->speed.angular.y;
	angular_speed_target[2] = msg->speed.angular.z;
}

void Regulator::updateCommandMsg()
{
	Regulator::computeFixes();

	geometry_msgs::Twist command_msg;

	command_msg.linear.x 	= linear_speed_target[0];
	command_msg.angular.z 	= u_yaw;

	command_pub.publish(command_msg);
}

void Regulator::computeFixes()
{
	double det, distanceBoatToLine, yaw_bar, yaw_radiale, e;

	if(isLine)
	{
		det         		= (x_target[0] - x_target_appoint[0])*(x[1] - x_target_appoint[1]) - (x[0] - x_target_appoint[0])*(x_target[1] - x_target_appoint[1]);
		distanceBoatToLine 	= det / (pow(pow(x_target[0] - x_target_appoint[0], 2) + pow(x_target[1] - x_target_appoint[1], 2), 0.5));
		yaw_bar     		= yaw_target + 0.4 * tanh(distanceBoatToLine);
	} else {
		yaw_bar     = atan2(x_target[0] - x[0], x_target[1] - x[1]);
	}

	e   = 2.0*atan(tan((yaw_bar - yaw)/2.0));

	if (abs(e) > full_left) {
		u_yaw = 1.0;
	} else {
		P = k_P*e;
		I += (1/25)*e;
		u_yaw = abs(P + k_I*I) >= 1 ? (abs(P + k_I*I)/(P + k_I*I))*1 : (P + k_I*I);
	}

	// Debug
	std_msgs::Float64MultiArray debug_msg;

	double debug_array[] = {yaw_bar, yaw, e};
	vector<double> debug_data = vector<double>(debug_array, debug_array + sizeof(debug_array) / sizeof(double) );

	debug_msg.data = debug_data;

	debug_pub.publish(debug_msg);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"regulator");
	Regulator regulator = Regulator();
	regulator.RunContinuously();
	return EXIT_SUCCESS;
}
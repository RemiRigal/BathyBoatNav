#include "../include/simulator.h"

using namespace std;

Simulator::Simulator()
{
	// Import datas
	Handle.getParam("/simulation/x_initial", converted_x[0]);
	Handle.getParam("/simulation/y_initial", converted_x[1]);
	Handle.getParam("/simulation/yaw_initial", yaw);
	Handle.getParam("/simulation/speed_initial", speed);

	// Robot state
	// robot_state_sub = Handle.subscribe("/robot_state_converted", 1000, &Simulator::updateRobotState, this);

	// Command
	command_sub = Handle.subscribe("/cons_helios", 1000, &Simulator::updateCommand, this);

	// Evolution
	robot_state_converted_evolved_pub = Handle.advertise<BathyBoatNav::robot_state>("/robot_state_converted_evolved", 1000);
	robot_state_raw_evolved_pub = Handle.advertise<BathyBoatNav::robot_state>("/robot_state_raw_evolved", 1000);

	// Coordinates conversion
	convert_coords_client = Handle.serviceClient<BathyBoatNav::gps_conversion>("/gps_converter");
}

Simulator::~Simulator()
{}

void Simulator::RunContinuously()
{
	// Loop
	ros::Rate loop_rate(25);

	while(ros::ok())
	{
		Simulator::evolution();

		Simulator::updateRobotStateRawEvolvedMsg();
		Simulator::updateRobotStateConvertedEvolvedMsg();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void Simulator::evolution()
{
	if (state == RUNNING)
	{
		converted_x[0]    += speed*(1/25)*sin(yaw);
		converted_x[1]    += speed*(1/25)*cos(yaw);

		yaw     += (1/25)*u_yaw;

		q.setRPY(0.0, 0.0, yaw);
	}

	BathyBoatNav::gps_conversion convert_coords_msg;

	convert_coords_msg.request.mode = 0;
	convert_coords_msg.request.long_or_x = converted_x[0];
	convert_coords_msg.request.lat_or_y = converted_x[1];

	if (convert_coords_client.call(convert_coords_msg))
	{
		x[0] = convert_coords_msg.response.long_or_x;
		x[1] = convert_coords_msg.response.lat_or_y;
	} else {
		ROS_ERROR("Failed to call gps converter");
	}
}

// void Simulator::updateRobotState(const BathyBoatNav::robot_state::ConstPtr& msg)
// {
// 	state = State(msg->state);

// 	x[0] = msg->pose.position.x;
// 	x[1] = msg->pose.position.y;
// 	x[2] = msg->pose.position.z;

// 	tf::Quaternion q_state(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
// 	tf::Matrix3x3 m(q);
// 	m.getRPY(roll, pitch, yaw);

// 	linear_speed[0] = msg->speed.linear.x;
// 	linear_speed[1] = msg->speed.linear.y;
// 	linear_speed[2] = msg->speed.linear.z;
// 	angular_speed[0] = msg->speed.angular.x;
// 	angular_speed[1] = msg->speed.angular.y;
// 	angular_speed[2] = msg->speed.angular.z;
// }

void Simulator::updateCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
	speed = msg->linear.x;
	u_yaw = msg->angular.z;
}

void Simulator::updateRobotStateConvertedEvolvedMsg()
{
	BathyBoatNav::robot_state robot_state_msg;

	robot_state_msg.header.stamp = ros::Time::now();

	robot_state_msg.pose.position.x = converted_x[0];
	robot_state_msg.pose.position.y = converted_x[1];
	robot_state_msg.pose.position.z = converted_x[2];
	robot_state_msg.pose.orientation.x = q.getX();
	robot_state_msg.pose.orientation.y = q.getY();
	robot_state_msg.pose.orientation.z = q.getZ();
	robot_state_msg.pose.orientation.w = q.getW();

	robot_state_converted_evolved_pub.publish(robot_state_msg);
}

void Simulator::updateRobotStateRawEvolvedMsg()
{
	BathyBoatNav::robot_state robot_state_msg;

	robot_state_msg.header.stamp = ros::Time::now();

	robot_state_msg.pose.position.x = x[0];
	robot_state_msg.pose.position.y = x[1];
	robot_state_msg.pose.position.z = x[2];
	robot_state_msg.pose.orientation.x = q.getX();
	robot_state_msg.pose.orientation.y = q.getY();
	robot_state_msg.pose.orientation.z = q.getZ();
	robot_state_msg.pose.orientation.w = q.getW();

	robot_state_raw_evolved_pub.publish(robot_state_msg);
}

int main(int argc, char** argv){
	ros::init(argc,argv,"simulator");
	Simulator simulator = Simulator();
	simulator.RunContinuously();
	return EXIT_SUCCESS;
}
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Int64.h"
#include "BathyBoatNav/new_state.h"
#include "BathyBoatNav/robot_state.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "maestro.h"

#include "../include/state.h"

using namespace std;

double u_throttle, u_yaw;

State state;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_throttle 	= msg->linear.x;
    u_yaw 		= msg->angular.z;
}

void init_servo(int fd)
{
	maestroSetTarget(fd, 0, 4000);
	maestroSetTarget(fd, 1, 4000);
	
	sleep(2);
}

void updateRobotState(const BathyBoatNav::robot_state::ConstPtr& msg)
{
	state = State(msg->state);
}

int main(int argc, char *argv [])
{
	int fd;

	string path;
	string channel;
	int gap;

	bool isSimulation;

	state = IDLE;

	double left_mot, right_mot;
	left_mot = 0;
	right_mot = 0;

	// Ros init

	ros::init(argc, argv, "pololu");
	ros::NodeHandle n;
    
	ros::Rate loop_rate(25);
    
    // Initials parameters
    
    n.param<string>("Path", path, "/dev/pololu_servo_serial");
    n.param<string>("Cons_channel", channel, "/cons_helios");
    n.param<int>("Turn_gap", gap, 1000);
    n.getParam("/isSimu", isSimulation);

	// Connection to Maestro
	if(!isSimulation)
	{
		if( (fd = maestroConnect(path.c_str())) == -1 )
		{
			ROS_INFO("Pololu connected");
			init_servo(fd);
		}
	}

	// Subscribe msgs

	ros::Subscriber cons_sub = n.subscribe(channel, 1000, chatCallback);

	ros::Subscriber robot_state_sub = n.subscribe("/robot_state_converted", 1000, updateRobotState);

	// Publisher

	ros::Publisher left_mot_pub = n.advertise<std_msgs::Int64>("/left_mot", 1000);
    std_msgs::Int64 left_mot_msgs;

	ros::Publisher right_mot_pub = n.advertise<std_msgs::Int64>("/right_mot", 1000);
    std_msgs::Int64 right_mot_msgs;

	while(ros::ok())
	{
		if(state == RUNNING)
		{
			double moteur_droite = u_throttle - u_yaw;
			double moteur_gauche = u_throttle + u_yaw;
			double moteur_droite2 = moteur_droite/max(max(1.0, moteur_droite), moteur_gauche);
			double moteur_gauche2 = moteur_gauche/max(max(1.0, moteur_droite), moteur_gauche);
			left_mot 	= 4000.0 + moteur_gauche2*3500.0;
			right_mot 	= 4000.0 + moteur_droite2*3500.0;
		} else {
			left_mot 	= 4000.0;
			right_mot 	= 4000.0;
		}

		left_mot_msgs.data = left_mot;
		left_mot_pub.publish(left_mot_msgs);

		right_mot_msgs.data = right_mot;
		right_mot_pub.publish(right_mot_msgs);

		if(!isSimulation)
		{
			maestroSetTarget(fd, 1, left_mot);
			maestroSetTarget(fd, 0, right_mot);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}

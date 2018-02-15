#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Int64.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "maestro.h"

using namespace std;

double u_throttle, u_yaw;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_throttle 	= 0.7;
    u_yaw 		= msg->angular.z;
}

void init_servo(int fd)
{
	maestroSetTarget(fd, 0, 4000);
	maestroSetTarget(fd, 1, 4000);
	
	sleep(2);
}


int main(int argc, char *argv [])
{
	int fd;

	string path;
	string channel;
	int gap;

	double left_mot, right_mot;
	left_mot = 0;
	right_mot = 0;

		// Ros init

	ros::init(argc, argv, "pololu");
	ros::NodeHandle n;
    
	ros::Rate loop_rate(25);
    
        // Initials parameters
    
	n.param<string>("Path", path, "/dev/pololu_servo_serial");
	n.param<string>("Cons_channel", channel, "cons_boat");
	n.param<int>("Turn_gap", gap, 500);

    	// Connection to Maestro

	if( (fd = maestroConnect(path.c_str())) == -1 )
	{
		perror("Unable to find Pololu");
		exit(1);
	}

	printf("Pololu connected\n");
	init_servo(fd);

		// Subscribe msgs

	ros::Subscriber cons_sub = n.subscribe(channel, 1000, chatCallback);

		// Publisher

	ros::Publisher left_mot_pub = n.advertise<std_msgs::Int64>("left_mot", 1000);
    std_msgs::Int64 left_mot_msgs;

	ros::Publisher right_mot_pub = n.advertise<std_msgs::Int64>("right_mot", 1000);
    std_msgs::Int64 right_mot_msgs;


	while(ros::ok())
	{

		left_mot 	= 4000 + u_throttle*(4000 - gap) + u_yaw*gap;
		right_mot 	= 4000 + u_throttle*(4000 - gap) - u_yaw*gap;

		left_mot_msgs.data = left_mot;
		left_mot_pub.publish(left_mot_msgs);

		right_mot_msgs.data = right_mot;
		right_mot_pub.publish(right_mot_msgs);

		maestroSetTarget(fd, 1, left_mot);
		maestroSetTarget(fd, 0, right_mot);

		ros::spinOnce();
		loop_rate.sleep();
	}

	exit(0);
}

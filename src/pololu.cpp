#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "geometry_msgs/Twist.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "maestro.h"

using namespace std;

double u_throttle, u_yaw;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_throttle 	= msg->linear.x;
    u_yaw 		= msg->angular.z;
}

void init_servo(int fd)
{
	maestroSetTarget(fd, 0, 6000);
	maestroSetTarget(fd, 1, 6000);
	sleep(2);
}


int main(int argc, char *argv [])
{
	int fd;

	string path;
	string channel;
	int gap;

	double left_mot, right_mot;

		// Ros init

	ros::init(argc, argv, "pololu");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(25);
    
        // Initials parameters
    
    n.param<string>("Path", path, "/dev/pololu");
    n.param<string>("Msg_name", channel, "/key_vel");
    n.param<int>("Turn_gap", gap, 100);

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

	while(ros::ok())
	{
		left_mot 	= 6000 + u_throttle*(2000 - gap) + u_yaw*gap;
		right_mot 	= 6000 + u_throttle*(2000 - gap) - u_yaw*gap;

		ROS_INFO("\nu_throttle -> %lf\nu_yaw -> %lf\nCons_pololu = (%lf, %lf)\n", u_throttle, u_yaw, left_mot, right_mot);

		maestroSetTarget(fd, 0, left_mot);
		maestroSetTarget(fd, 1, right_mot);

		ros::spinOnce();
        loop_rate.sleep();
	}

	exit(0);
}
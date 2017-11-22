#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "geometry_msgs/Twist.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "maestro.h"

using namespace std;

double u_yaw;
double left_mot, right_mot;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_yaw 	= msg->angular.z;
}


int main(int argc, char *argv [])
{
	ros::init(argc, argv, "pololu");
    ros::NodeHandle n;
    
        // Parametres initiaux
    //n.param<double>("Pos_x", x[0], 0);

    ros::Rate loop_rate(10);


    	// Connection to Maestro
	int fd = maestroConnect("/dev/ttyACM0");
	printf("Pololu connected\n");

	maestroSetTarget(fd, 0, 8000);
	maestroSetTarget(fd, 1, 8000);

	sleep(2);

	maestroSetTarget(fd, 0, 4000);
	maestroSetTarget(fd, 1, 4000);

	sleep(2);

	maestroSetTarget(fd, 0, 6000);
	maestroSetTarget(fd, 1, 6000);

		// Subscribe msgs
    ros::Subscriber cons_sub = n.subscribe("cons_boat", 1000, chatCallback);

	while(ros::ok())
	{
		left_mot 	= 6000 + u_yaw*2000;
		right_mot 	= 6000 - u_yaw*2000;

		ROS_INFO("Info -> %lf | (%lf, %lf)\n", u_yaw, left_mot, right_mot);

		maestroSetTarget(fd, 0, left_mot);
		maestroSetTarget(fd, 1, right_mot);

		ros::spinOnce();
        loop_rate.sleep();
	}

	exit(1);
}
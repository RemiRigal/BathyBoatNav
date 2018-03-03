#include <iostream>
#include <string>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Pose.h"

using namespace std;

double latitude, longitude;
double x_lambert, y_lambert;

void chatCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    latitude   = msg->latitude;
    longitude  = msg->longitude;
}

void convert()
{
	x_lambert = latitude;
	y_lambert = longitude;
}

int main(int argc, char** argv)
{
	string input_msg;
	string output_msg;

        // Ros init

    ros::init(argc, argv, "convert2Lambert");
    ros::NodeHandle n;

    ros::Rate loop_rate(25);

        // Initials parameters

    n.param<string>("Input_msg", 	input_msg, 	"unknown");
    n.param<string>("Output_msg", 	output_msg, "unknown");

        // Subscribe

    ros::Subscriber input_sub = n.subscribe(input_msg, 1000, chatCallback);

        // Publish

    ros::Publisher output_pub = n.advertise<geometry_msgs::Pose>(output_msg, 1000);
    geometry_msgs::Pose lambert_msg;
    
    
    while(ros::ok())
    {      
    	convert();
    	
    	lambert_msg.position.x = x_lambert;
    	lambert_msg.position.y = y_lambert;

    	output_pub.publish(lambert_msg);
    	
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
#ifndef _GPS_CONVERTER_HH
#define _GPS_CONVERTER_HH

#include <proj_api.h>
#include <stdio.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Twist.h"
#include "BathyBoatNav/gps_conversion.h"
#include <string>

class Converter {
private:
    ros::NodeHandle Handle;
    ros::Subscriber gps_sub;
    ros::Subscriber angle_sub;
    ros::Publisher pub;
    ros::ServiceServer converterSrv;
    geometry_msgs::Twist pose;
    BathyBoatNav::gps_conversion::Response Lambert_to_latlong(double x, double y);
    BathyBoatNav::gps_conversion::Response Latlong_to_lambert(double x, double y);
    bool convertService(BathyBoatNav::gps_conversion::Request &req, BathyBoatNav::gps_conversion::Response &res);

    std::string input_GPS_msg;
    std::string input_yaw_msg;
    std::string output_msg;

    void convert2LambertCallback(sensor_msgs::NavSatFix::ConstPtr& msg);
    void angleCallback(geometry_msgs::Vector3Stamped::ConstPtr& msg);

public:
    Converter();
	virtual ~Converter();
    void RunContinuously();
};

#endif

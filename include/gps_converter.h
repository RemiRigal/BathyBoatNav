#ifndef _GPS_CONVERTER_HH
#define _GPS_CONVERTER_HH

#include <proj_api.h>
#include <stdio.h>
#include "ros/ros.h"
#include "BathyBoatNav/gps_conversion.h"


class Converter {
private:
    ros::NodeHandle Handle;
    ros::ServiceServer converterSrv;
    BathyBoatNav::gps_conversion::Response Lambert_to_latlong(double x, double y);
    BathyBoatNav::gps_conversion::Response Latlong_to_lambert(double x, double y);
    bool convertService(BathyBoatNav::gps_conversion::Request &req, BathyBoatNav::gps_conversion::Response &res);

public:
    void Prepare();
    void RunContinuously();
};

#endif

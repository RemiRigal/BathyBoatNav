#ifndef _GPS_CONVERTER_H
#define _GPS_CONVERTER_H

#include <proj_api.h>
#include <stdio.h>
#include <string>
#include "ros/ros.h"

#include "BathyBoatNav/gps_conversion.h"

class Converter {
    public:
        Converter();
        virtual ~Converter();
        void RunContinuously();
        
    private:
        ros::NodeHandle Handle;

        ros::ServiceServer converterSrv;

        BathyBoatNav::gps_conversion::Response Lambert_to_latlong(double x, double y);
        BathyBoatNav::gps_conversion::Response Latlong_to_lambert(double x, double y);

        bool convertService(BathyBoatNav::gps_conversion::Request &req, BathyBoatNav::gps_conversion::Response &res);

        projPJ pj_lambert, pj_latlong;


};

#endif

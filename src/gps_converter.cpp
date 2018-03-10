#include "../include/gps_converter.h"

using namespace std;

Converter::Converter()
{
	Handle.param<string>("Input_GPS_msg", input_GPS_msg, "nav");
	Handle.param<string>("Input_yaw_msg", input_yaw_msg, "imu_attitude");
	Handle.param<string>("Output_msg", output_msg, "gps_angle_boat");

	converterSrv = Handle.advertiseService("gps_converter", &Converter::convertService, this);
	//gps_sub = Handle.subscribe(input_GPS_msg, 1000, &Converter::convert2LambertCallback, this); 
	//angle_sub = Handle.subscribe(input_yaw_msg, 1000, &Converter::angleCallback, this); 
	//pub = Handle.advertise<geometry_msgs::Twist>(output_msg, 100); 
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

Converter::~Converter()
{}


void Converter::convert2LambertCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	BathyBoatNav::gps_conversion::Response res = this->Latlong_to_lambert(msg->latitude, msg->latitude);
	pose.linear.x = res.converted_x;
	pose.linear.y = res.converted_y;
}

void Converter::angleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	pose.angular.z = msg->vector.z ;
}

bool Converter::convertService(BathyBoatNav::gps_conversion::Request &req, BathyBoatNav::gps_conversion::Response &res)
{
	if (req.mode == 0)
	{
		res = this->Lambert_to_latlong(req.gps_x, req.gps_y);
		return true;
	} else if (req.mode == 1) {
		res = this->Latlong_to_lambert(req.gps_x, req.gps_y);
		return true;
	}
	return false;
}

BathyBoatNav::gps_conversion::Response Converter::Lambert_to_latlong(double x, double y)
{
	BathyBoatNav::gps_conversion::Response res;
	projPJ pj_lambert, pj_latlong;

	if (!(pj_lambert = pj_init_plus("+proj=lcc +lat_1=49 +lat_2=44 +lat_0=46.5 +lon_0=3 +x_0=700000 +y_0=6600000 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ")))
	{
		printf("error lambert \n");
		exit(1);
	}
	
	if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
	{
		printf("error latlong \n");
		exit(1);
	}

	pj_transform(pj_lambert, pj_latlong, 1, 1, &x, &y, NULL );
	x *= RAD_TO_DEG;
	y *= RAD_TO_DEG;
	//printf("lng: %lf \nlat: %lf\n", x, y);
	res.converted_x = x;
	res.converted_y = y;
	return res;
}

BathyBoatNav::gps_conversion::Response Converter::Latlong_to_lambert(double x, double y)
{
	BathyBoatNav::gps_conversion::Response res;
	projPJ pj_lambert, pj_latlong;

	if (!(pj_lambert = pj_init_plus("+proj=lcc +lat_1=49 +lat_2=44 +lat_0=46.5 +lon_0=3 +x_0=700000 +y_0=6600000 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ")))
	{
		printf("error lambert \n");
		exit(1);
	}

	if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
	{
		printf("error latlong \n");
		exit(1);
	}
	
	x *= DEG_TO_RAD;
	y *= DEG_TO_RAD;
	pj_transform(pj_latlong, pj_lambert, 1, 1, &x, &y, NULL );
	//printf("X: %lf \nY: %lf\n", x, y);
	res.converted_x = x;
	res.converted_y = y;
	return res;
}

void Converter::RunContinuously()
{
	ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		//pub.publish(pose);

		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "gps_converter");
	Converter converter;
	converter.RunContinuously();
	return (0);
}

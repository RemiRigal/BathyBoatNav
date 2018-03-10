#include "../include/gps_converter.h"

using namespace std;

Converter::Converter()
{
	converterSrv = Handle.advertiseService("/gps_converter", &Converter::convertService, this);

	if (!(pj_lambert = pj_init_plus("+proj=lcc +lat_1=49 +lat_2=44 +lat_0=46.5 +lon_0=3 +x_0=700000 +y_0=6600000 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ")))
	{
		printf("Error Lambert \n");
		exit(1);
	}
	
	if (!(pj_latlong = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
	{
		printf("Error LatLong \n");
		exit(1);
	}
}

Converter::~Converter()
{}

bool Converter::convertService(BathyBoatNav::gps_conversion::Request &req, BathyBoatNav::gps_conversion::Response &res)
{
	if (req.mode == 0)
	{
		res = this->Lambert_to_latlong(req.long_or_x, req.lat_or_y);
		return true;
	} else if (req.mode == 1) {
		res = this->Latlong_to_lambert(req.long_or_x, req.lat_or_y);
		return true;
	}
	return false;
}

BathyBoatNav::gps_conversion::Response Converter::Lambert_to_latlong(double x, double y)
{
	BathyBoatNav::gps_conversion::Response res;

	pj_transform(pj_lambert, pj_latlong, 1, 1, &x, &y, NULL );

	x *= RAD_TO_DEG;
	y *= RAD_TO_DEG;

	res.long_or_x = x;
	res.lat_or_y = y;

	return res;
}

BathyBoatNav::gps_conversion::Response Converter::Latlong_to_lambert(double x, double y)
{
	BathyBoatNav::gps_conversion::Response res;
	
	x *= DEG_TO_RAD;
	y *= DEG_TO_RAD;

	pj_transform(pj_latlong, pj_lambert, 1, 1, &x, &y, NULL );

	res.long_or_x = x;
	res.lat_or_y = y;

	return res;
}

void Converter::RunContinuously()
{
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
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
#ifndef _MAESTRO_H
#define _MAESTRO_H

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
 
#include <ros/ros.h>

// Services
#include "BathyBoatNav/get_pololu.h"
#include "BathyBoatNav/set_pololu.h"

class Maestro
{
	public:
		Maestro();
		~Maestro();
		void RunContinuously();

	private:
		int pololu_fd;
		std::string pololu_path;

		// Getting position
		ros::ServiceServer getting_position_srv;
		bool parseGetPosition(BathyBoatNav::get_pololu::Request &req, BathyBoatNav::get_pololu::Response &res);
		int getPosition(unsigned char channel);

		// Setting target
		ros::ServiceServer setting_target_srv;
		bool parseSetTarget(BathyBoatNav::set_pololu::Request &req, BathyBoatNav::set_pololu::Response &res);
		bool setTarget(unsigned char channel, unsigned short target);
};

#endif
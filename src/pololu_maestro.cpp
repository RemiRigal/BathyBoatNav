#include "../include/pololu_maestro.h"

using namespace std;

Maestro::Maestro()
{
	ros::NodeHandle Handle;

	// Import datas
	Handle.getParam("/ros/pololu/path", pololu_path);

	if ( (pololu_fd = open(pololu_path.c_str(), O_RDWR | O_NOCTTY) ) == -1)
	{
		ROS_INFO("Pololu not found on path : %s", pololu_path.c_str());
	}

	struct termios options;
	tcgetattr(pololu_fd, &options);
	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tcsetattr(pololu_fd, TCSANOW, &options);

	// Service for setting target
	getting_position_srv 	= Handle.advertiseService("/get_pos_pololu", &Maestro::parseGetPosition, this);;
	setting_target_srv 		= Handle.advertiseService("/set_target_pololu", &Maestro::parseSetTarget, this);;
}

Maestro::~Maestro()
{
	close(pololu_fd);
}

void Maestro::RunContinuously()
{
	// Loop
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int Maestro::getPosition(unsigned char channel)
{
	unsigned char command[] = {0x90, channel};
	if(write(pololu_fd, command, sizeof(command)) == -1)
	{
		ROS_INFO("Error sending command to get position on channel %c", channel);
		return -1;
	}

	unsigned char response[2];
	if(read(pololu_fd,response,2) != 2)
	{
		ROS_INFO("Error receving response for getting position on channel %c", channel);
		return -1;
	}

	return response[0] + 256*response[1];
}
 
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
bool Maestro::setTarget(unsigned char channel, unsigned short target)
{
	unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
	if (write(pololu_fd, command, sizeof(command)) == -1)
	{
		ROS_INFO("Error sending command to set target %d on channel %c", target, channel);
		return false;
	}
	return true;
}
 
bool Maestro::parseSetTarget(BathyBoatNav::set_pololu::Request &req, BathyBoatNav::set_pololu::Response &res)
{
	res.success = Maestro::setTarget(req.channel, req.target);

	return res.success;
}

bool Maestro::parseGetPosition(BathyBoatNav::get_pololu::Request &req, BathyBoatNav::get_pololu::Response &res)
{
	res.value = Maestro::getPosition(req.channel);

	if (res.value == -1)
	{
		res.success = true;
	} else {
		res.success = false;
	}

	return res.success;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"maestro");
	Maestro maestro = Maestro();
	maestro.RunContinuously();
	return EXIT_SUCCESS;
}
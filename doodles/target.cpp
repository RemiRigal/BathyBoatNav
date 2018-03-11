#include "../include/target.h"

using namespace std;

Target::Target()
{
	
}

Target::~Target()
{}

void Target::RunContinuously()
{
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char** argv){
	ros::init(argc,argv,"target_manager");
	Target target = Target();
	target.RunContinuously();
	return EXIT_SUCCESS;
}
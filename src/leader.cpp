#include "../include/leader.h"

using namespace std;

Leader::Leader()
{
	// Import datas
	Handle.getParam("/isSimu", isSimulation);
	Handle.getParam("/ros/target/accept_dist", accept_dist);
	Handle.getParam("/ros/regulation/k_P", k_P);
	Handle.getParam("/ros/regulation/k_I", k_I);
	Handle.getParam("/ros/regulation/k_D", k_D);
	Handle.getParam("/ros/regulation/initial_speed", linear_speed_target[0]);
	
	// Robot state
	robot_state_raw_pub = Handle.advertise<BathyBoatNav::robot_state>("/robot_state_raw", 1000);

	// Robot state
	robot_state_converted_pub = Handle.advertise<BathyBoatNav::robot_state>("/robot_state_converted", 1000);

	// Robot target
	robot_target_pub = Handle.advertise<BathyBoatNav::robot_target>("/robot_target", 1000);

	// Parsing TCP inputs
	serverInputs = Handle.advertiseService("/TCP_inputs", &Leader::parseCommand, this);

	// Change state of FSM
	changeStateSrv = Handle.serviceClient<BathyBoatNav::new_state>("changeStateSrv");

	// New mission
	mision_path_client = Handle.serviceClient<BathyBoatNav::message>("new_mission");

	// Next goal
	next_goal_client = Handle.serviceClient<BathyBoatNav::next_goal>("next_goal");

	// Coordinates conversion
	convert_coords_client = Handle.serviceClient<BathyBoatNav::gps_conversion>("/gps_converter");

	// Evolution if simulation
	if (isSimulation)
	{
		ROS_INFO("Simulation mode");
		robot_state_converted_evolved_sub = Handle.subscribe("/evolved_robot_state_converted", 1000, &Leader::updateRobotStateConvertedEvolved, this);
		robot_state_raw_evolved_sub = Handle.subscribe("/evolved_robot_state_raw", 1000, &Leader::updateRobotStateRawEvolved, this);
	} else {
		ROS_INFO("Normal mode");
		sbg_imu_pos_sub = Handle.subscribe("output/log_ekf_nav", 1000, &Leader::updateImuPosition, this);
		sbg_imu_vel_sub = Handle.subscribe("output/log_ship_motion", 1000, &Leader::updateImuVelocity, this);
		sbg_gps_pos_sub = Handle.subscribe("output/log_gps1_pos", 1000, &Leader::updateGpsPosition, this);
		sbg_gps_vel_sub = Handle.subscribe("output/log_gps1_vel", 1000, &Leader::updateGpsVelocity, this);
	}

}

Leader::~Leader()
{}

void Leader::RunContinuously()
{
	// Init state
	Leader::setState("IDLE");

	// Loop
	ros::Rate loop_rate(25);
	while(ros::ok())
	{
		if(state == RUNNING)
		{
			Leader::checkIfTargetValidated();
		}

		Leader::updateRobotStateMsg();
		Leader::updateRobotTargetMsg();

		ros::spinOnce();
		loop_rate.sleep();
	}
}

bool Leader::setState(string state_str)
{
	new_state_msg.request.state = state_str;
	if (changeStateSrv.call(new_state_msg))
	{
		if(!new_state_msg.response.success)
		{
			ROS_ERROR("FSM failed to change state");
			return false;
		}
		state = State(new_state_msg.response.state);
	} else {
		ROS_ERROR("Failed to call FSM");
		return false;
	}

	return true;
}

bool Leader::parseCommand(BathyBoatNav::message::Request &req, BathyBoatNav::message::Response &res)
{
	string msg = req.message;
	vector<string> split_msg;
	string key_msg;

	bool error = true;

	boost::split(split_msg, msg, boost::is_any_of("|"));

	int sizeVector = split_msg.size();

	if(sizeVector != 0)
	{
		key_msg = split_msg[0];
		if(key_msg == "MISSION")
		{
			if(sizeVector == 2)
			{
				error = ! Leader::changeMission(split_msg[1]);
			} else {
				ROS_INFO("Mission message canvas is \"MISSION|name_of_mission.json\" but received \"%s\"", msg.c_str());
			}
		} else if(key_msg == "FACTOR") {
			if(sizeVector == 3)
			{
				Leader::changePID(atof(split_msg[1].c_str()), atof(split_msg[2].c_str()));
				error = false;
			} else {
				ROS_INFO("PID message canvas is \"FACTOR|k_I|k_P\" but received \"%s\"", msg.c_str());
			}
		} else if(key_msg == "SPEED") {
			if(sizeVector == 2)
			{
				Leader::changeSpeed(atof(split_msg[1].c_str()));
				error = false;
			} else {
				ROS_INFO("Speed message canvas is \"SPEED|coeffSPD\" but received \"%s\"", msg.c_str());
			}
		} else if(key_msg == "RESUME" || key_msg == "PAUSE" || key_msg == "RTL" || key_msg == "STOP" || key_msg == "EMERGENCY") {
			if(sizeVector == 1)
			{
				error = ! Leader::setState(key_msg);
			} else {
				ROS_INFO("State message canvas is \"STATE\" but received \"%s\"", msg.c_str());
			}
		} else {
			ROS_WARN("Received message %s but parsing give no meaning to it.", msg.c_str());
		}
	}

	res.success = !error;

	return res.success;
}

bool Leader::changeMission(string path)
{
	ROS_INFO("New mission path %s", path.c_str());

	mission_path_msg.request.message = path;

	if(mision_path_client.call(mission_path_msg))
	{                
		if(mission_path_msg.response.success)
		{
			ROS_INFO("Mission parsed");
			mission_finished = false;
			Leader::askForNewWaypoints();
			Leader::setState("PAUSE");
		} else {
			ROS_WARN("Mission parsing failed");
		}
	} else {
		ROS_WARN("Call to mission interpreter failed");
	}

	return true;
}

void Leader::changePID(double P, double I)
{
	ROS_INFO("Change of PID factor to k_P = %lf and k_I = %lf", P, I);
	k_P = P;
	k_I = I;
}

void Leader::changeSpeed(double speed)
{
	ROS_INFO("Change of speed factor to %lf", speed);
	linear_speed_target[0] = speed;
}

void Leader::checkIfTargetValidated()
{
	double dist = pow(pow(x_target[0] - converted_x[0],2) + pow(x_target[1] - converted_x[1],2), 0.5);
	if(dist <= accept_dist)
	{
		ROS_INFO("Asking for new target");
		Leader::askForNewWaypoints();
	}
}

void Leader::askForNewWaypoints()
{
	double yaw_radiale;

    if (next_goal_client.call(next_goal_msg))
    {
        if( ! mission_finished )
        {
            isLine       	= next_goal_msg.response.isRadiale;
            x_target[0]     = next_goal_msg.response.latitude[0];
            x_target[1]     = next_goal_msg.response.longitude[0];
            id              = next_goal_msg.response.id;

            if(isLine)
            {
                x_target_appoint[0] = next_goal_msg.response.latitude[1];
                x_target_appoint[1] = next_goal_msg.response.longitude[1];
            }    
            yaw_radiale = isLine ? atan2(x_target[0] - x_target_appoint[0], x_target[1] - x_target_appoint[1]) : 0.0;

            q_target.setRPY(0.0, 0.0, yaw_radiale);

            ROS_INFO("Target -> %s | (%lf, %lf)", isLine ? "Radiale" : "Waypoint", x_target[0], x_target[1]);
            mission_finished = next_goal_msg.response.isLast;
        } else {
        	ROS_INFO("Mission finished. IDLE state.");
        	Leader::setState("IDLE");
        }

    } else{
        ROS_ERROR("Failed to call next goal service");
    }
}

void Leader::updateRobotStateMsg()
{
	BathyBoatNav::robot_state robot_state_msg;

	robot_state_msg.header.stamp = ros::Time::now();

	robot_state_msg.state = state;
	robot_state_msg.gps_status = gps_status;

	robot_state_msg.pose.position.x = x[0];
	robot_state_msg.pose.position.y = x[1];
	robot_state_msg.pose.position.z = x[2];
	robot_state_msg.pose.orientation.x = q.getX();
	robot_state_msg.pose.orientation.y = q.getY();
	robot_state_msg.pose.orientation.z = q.getZ();
	robot_state_msg.pose.orientation.w = q.getW();

	robot_state_msg.speed.linear.x = linear_speed[0];
	robot_state_msg.speed.linear.y = linear_speed[1];
	robot_state_msg.speed.linear.z = linear_speed[2];
	robot_state_msg.speed.angular.x = angular_speed[0];
	robot_state_msg.speed.angular.y = angular_speed[1];
	robot_state_msg.speed.angular.z = angular_speed[2];

	robot_state_msg.pid.k_P = k_P;
	robot_state_msg.pid.k_I = k_I;
	robot_state_msg.pid.k_D = k_D;

	double batt_array[] = {0.50, 0.60, 0.40};
	batt = vector<double>(batt_array, batt_array + sizeof(batt_array) / sizeof(double) );

	robot_state_msg.batteries.data = batt;

	robot_state_raw_pub.publish(robot_state_msg);

	Leader::updateRobotStateConvertedMsg();
}

void Leader::updateRobotStateConvertedMsg()
{
	BathyBoatNav::robot_state robot_state_msg;
	BathyBoatNav::gps_conversion convert_coords_msg;

	if (!isSimulation)
	{
		convert_coords_msg.request.mode = 1;
		convert_coords_msg.request.long_or_x = x[0];
		convert_coords_msg.request.lat_or_y = x[1];

		if (convert_coords_client.call(convert_coords_msg))
		{
			converted_x[0] = convert_coords_msg.response.long_or_x;
			converted_x[1] = convert_coords_msg.response.lat_or_y;
		} else {
			ROS_ERROR("Failed to call gps converter");
		}
	}

	robot_state_msg.header.stamp = ros::Time::now();

	robot_state_msg.state = state;
	robot_state_msg.gps_status = gps_status;

	robot_state_msg.pose.position.x = converted_x[0];
	robot_state_msg.pose.position.y = converted_x[1];
	robot_state_msg.pose.position.z = converted_x[2];
	robot_state_msg.pose.orientation.x = q.getX();
	robot_state_msg.pose.orientation.y = q.getY();
	robot_state_msg.pose.orientation.z = q.getZ();
	robot_state_msg.pose.orientation.w = q.getW();

	robot_state_msg.speed.linear.x = linear_speed[0];
	robot_state_msg.speed.linear.y = linear_speed[1];
	robot_state_msg.speed.linear.z = linear_speed[2];
	robot_state_msg.speed.angular.x = angular_speed[0];
	robot_state_msg.speed.angular.y = angular_speed[1];
	robot_state_msg.speed.angular.z = angular_speed[2];

	robot_state_msg.pid.k_P = k_P;
	robot_state_msg.pid.k_I = k_I;
	robot_state_msg.pid.k_D = k_D;

	double batt_array[] = {50.0, 60.0, 40.0};
	batt = vector<double>(batt_array, batt_array + sizeof(batt_array) / sizeof(double) );

	robot_state_msg.batteries.data = batt;

	robot_state_converted_pub.publish(robot_state_msg);
}

void Leader::updateRobotTargetMsg()
{
	BathyBoatNav::robot_target robot_target_msg;

	robot_target_msg.header.stamp = ros::Time::now();
	
	robot_target_msg.isLine = isLine;

	robot_target_msg.pose.position.x = x_target[0];
	robot_target_msg.pose.position.y = x_target[1];
	robot_target_msg.pose.position.z = x_target[2];
	robot_target_msg.pose.orientation.x = q_target.getX();
	robot_target_msg.pose.orientation.y = q_target.getY();
	robot_target_msg.pose.orientation.z = q_target.getZ();
	robot_target_msg.pose.orientation.w = q_target.getW();

	robot_target_msg.pose_appoint.position.x = x_target_appoint[0];
	robot_target_msg.pose_appoint.position.y = x_target_appoint[1];
	robot_target_msg.pose_appoint.position.z = x_target_appoint[2];
	robot_target_msg.pose_appoint.orientation.x = q_target_appoint.getX();
	robot_target_msg.pose_appoint.orientation.y = q_target_appoint.getY();
	robot_target_msg.pose_appoint.orientation.z = q_target_appoint.getZ();
	robot_target_msg.pose_appoint.orientation.w = q_target_appoint.getW();

	robot_target_msg.speed.linear.x = linear_speed_target[0];
	robot_target_msg.speed.linear.y = linear_speed_target[1];
	robot_target_msg.speed.linear.z = linear_speed_target[2];
	robot_target_msg.speed.angular.x = angular_speed_target[0];
	robot_target_msg.speed.angular.y = angular_speed_target[1];
	robot_target_msg.speed.angular.z = angular_speed_target[2];

	robot_target_pub.publish(robot_target_msg);
}

void Leader::updateRobotStateConvertedEvolved(const BathyBoatNav::robot_state::ConstPtr& msg)
{
	converted_x[0] = msg->pose.position.x;
	converted_x[1] = msg->pose.position.y;
	converted_x[2] = msg->pose.position.z;

	tf::Quaternion q_evolved(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

	q = q_evolved;
}

void Leader::updateRobotStateRawEvolved(const BathyBoatNav::robot_state::ConstPtr& msg)
{
	x[0] = msg->pose.position.x;
	x[1] = msg->pose.position.y;
	x[2] = msg->pose.position.z;

	tf::Quaternion q_evolved(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

	q = q_evolved;
}

void Leader::updateImuPosition(const sbg_driver::SbgEkfNav::ConstPtr& msg)
{
	x[0] = msg->position.x;
	x[1] = msg->position.y;
	x[2] = msg->position.z;
}

void Leader::updateImuVelocity(const sbg_driver::SbgShipMotion::ConstPtr& msg)
{
	linear_speed[0] = msg->velocity.x;
}

void Leader::updateGpsPosition(const sbg_driver::SbgGpsPos::ConstPtr& msg)
{

}

void Leader::updateGpsVelocity(const sbg_driver::SbgGpsVel::ConstPtr& msg)
{

}

int main(int argc, char** argv){
	ros::init(argc,argv,"leader");
	Leader leader = Leader();
	sleep(2);
	leader.RunContinuously();
	return EXIT_SUCCESS;
}
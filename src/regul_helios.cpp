#include <iostream>
#include <string>
#include <cmath>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "BathyBoatNav/next_goal.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;

const double Pi = 3.14159265358979323846;

double gis;

double y_target, x_target;
double y_target_start_line, x_target_start_line;
bool isRadiale;
int still_n_mission;

double x_boat, y_boat;
double roll, pitch, yaw_boat, yaw_radiale;

double dist_max;

double u_yaw;
double u_vitesse;

double dist;

string name;
string state;

void computeDistance()
{
    dist = pow(pow(x_target - x_boat,2) + pow(y_target - y_boat,2), 0.5);
}

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    x_boat = msg->linear.x;
    y_boat = msg->linear.y;
    yaw_boat = msg->linear.z;
}

int main(int argc, char** argv)
{
    int num_waypoints = 1;
    double e;
    double zone_morte;
    double full_left;
    double det;
    double k;
    double dist_line;

    u_yaw = 0;
    u_vitesse = 0;

        // Ros init

    ros::init(argc, argv, "regul_helios");
    ros::NodeHandle n;

    ros::Rate loop_rate(25);

	        // Initials parameters

    n.param<string>("Name_boat", name, "helios");
    n.param<double>("Accept_gap", dist_max, 10.0);
    n.param<double>("Coeff", k, 0.5);
    n.param<double>("Zone_morte", zone_morte, 0.05);
    n.param<double>("Full_left", full_left, 2.5);
        // Info boat
/*
    ros::Subscriber gps_sub     = n.subscribe("nav",   1000, gpsCallback);
    ros::Subscriber angle_sub   = n.subscribe("imu",   1000, angleCallback);
*/
    ros::Subscriber data_sub   = n.subscribe("gps_angle_boat",   1000, callback);

        // Consigne

    ros::Publisher cons_pub = n.advertise<geometry_msgs::Twist>("cons_boat", 1000);
    geometry_msgs::Twist cons_msgs;

    ros::Publisher debug_pub = n.advertise<geometry_msgs::Twist>("debug_boat", 1000);
    geometry_msgs::Twist debug_msgs;

        // New GPS client

    ros::ServiceClient next_goal_client = n.serviceClient<BathyBoatNav::next_goal>("next_goal");
    BathyBoatNav::next_goal next_goal_msg;

	if(next_goal_client.call(next_goal_msg))
	{                
		if( (int)sizeof(next_goal_msg.response.latitude) != 0 )
        {
            isRadiale           = next_goal_msg.response.isRadiale;
            x_target     = next_goal_msg.response.latitude[0];
            y_target    = next_goal_msg.response.longitude[0];
            still_n_mission     = next_goal_msg.response.remainingMissions;
    		             
    		if(isRadiale)
            {
                x_target_start_line = next_goal_msg.response.latitude[1];
                y_target_start_line = next_goal_msg.response.longitude[1];
                
                yaw_radiale = atan2(x_target - x_target_start_line, y_target - y_target_start_line);
            }

        } else {
            state = "IDLE";
        }
	} else {
		ROS_WARN("Init call failed");
	}

    
    while(ros::ok())
    {
        computeDistance();

        if(dist < dist_max)
        {
            if (next_goal_client.call(next_goal_msg))
            {
                if( (int)sizeof(next_goal_msg.response.latitude) != 0 )
                {
                    isRadiale           = next_goal_msg.response.isRadiale;
                    x_target     = next_goal_msg.response.latitude[0];
                    y_target    = next_goal_msg.response.longitude[0];
                    still_n_mission     = next_goal_msg.response.remainingMissions;
                    if(isRadiale)
                    {
                        x_target_start_line = next_goal_msg.response.latitude[1];
                        y_target_start_line = next_goal_msg.response.longitude[1];
                        
                        yaw_radiale = atan2(x_target - x_target_start_line, y_target - y_target_start_line);
                    }
                } else {
                    state = "IDLE";
                }
                num_waypoints ++;
            } else{
                ROS_ERROR("Failed to call service");
            }
        }

        if(isRadiale)
        {
            //gis     = atan2(y_target - y_boat, x_target - x_boat);
            //det     = sin(yaw_radiale - yaw_boat - gis) * dist;
            //gis     = (yaw_radiale - atan(det)/2.0) - yaw_boat; 

            det         = (x_target - x_target_start_line)*(y_boat - y_target_start_line) - (x_boat - x_target_start_line)*(y_target - y_target_start_line);
            dist_line   = det / (pow(pow(x_target - x_target_start_line,2) + pow(y_target - y_target_start_line,2), 0.5));
            gis         = yaw_radiale * tanh(dist_line);
        } else {
            gis     = atan2(x_target - x_boat, y_target - y_boat);
        }

        e   = 2.0*atan(tan((gis - yaw_boat)/2.0));
        if(abs(e) < zone_morte)
        {
            u_yaw = 0.0;
        } else if (abs(e) > full_left) {
            u_yaw = 0.8;
        } else {
            u_yaw = k*e;
        }

        cons_msgs.angular.z = u_yaw;
        cons_msgs.linear.x  = 0;
        cons_pub.publish(cons_msgs);

        debug_msgs.linear.x = x_target;
        debug_msgs.linear.y = y_target;
        debug_msgs.linear.z = dist;
        debug_msgs.angular.x = gis;
        debug_msgs.angular.y = e;
        debug_msgs.angular.z = num_waypoints;

        debug_pub.publish(debug_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

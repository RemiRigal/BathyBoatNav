#include <iostream>
#include <string>
#include <cmath>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Pose2D.h"
#include "BathyBoatNav/next_goal.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;

const double Pi = 3.14159265358979323846;

double gis;

double latitude_target, longitude_target;
bool isRadiale;
int still_n_mission;

double latitude_boat, longitude_boat;
double roll, pitch, yaw_boat, yaw_radiale;

double dist_max;

double u_yaw;
double u_vitesse;

double dist;

string name;
string state;

void computeDistance()
{
    dist = pow(pow(latitude_target - latitude_boat,2) + pow(longitude_target - longitude_boat,2), 0.5);
}

/*
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    latitude_boat   = msg->latitude;
    longitude_boat  = msg->longitude;
}

void angleCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q_boat;

    q_boat.setX(msg->orientation.x);
    q_boat.setY(msg->orientation.y);
    q_boat.setZ(msg->orientation.z);
    q_boat.setW(msg->orientation.w);

    tf::Matrix3x3(q_boat).getRPY(roll, pitch, yaw_boat);
}
*/

void callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    latitude_boat   = msg->x;
    longitude_boat  = msg->y;
    yaw_boat        = msg->theta;
}

int main(int argc, char** argv)
{
    double e;

    u_yaw = 0;
    u_vitesse = 0;
	
    bool init = false;

        // Ros init

    ros::init(argc, argv, "regul_helios");
    ros::NodeHandle n;

    ros::Rate loop_rate(25);

    double compt = ros::Time::now().toSec();


	        // Initials parameters

    n.param<string>("Name_boat", name, "helios");
    n.param<double>("Accept_gap", dist_max, 3.0);

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
	ROS_WARN("Init call success");
} else {
	ROS_WARN("Init call failed");
}

    
    while(ros::ok())
    {
        double det;

        computeDistance();

        if(isRadiale)
        {
            gis     = atan2(longitude_target - longitude_boat, latitude_target - latitude_boat);
            det     = sin(yaw_radiale - yaw_boat - gis) * dist;
            gis     = (yaw_radiale - atan(det)/2.0) - yaw_boat;  
        } else {
            gis     = atan2(longitude_target - longitude_boat, latitude_target - latitude_boat);
        }

        e   = 2*atan(tan(gis/2));
        if(e < 0.0005 && e > -0.0005)
        {
            u_yaw = 0.0;
        } else if (e > 2.5 || e < -2.5) {
            u_yaw = 0.8;
        } else {
            u_yaw = k*atan(e);
        }

        cons_msgs.angular.z = u_yaw;
        cons_msgs.linear.x  = 0;
        cons_pub.publish(cons_msgs);

        debug_msgs.linear.x = latitude_target;
        debug_msgs.linear.y = longitude_target;
        debug_msgs.linear.z = dist;
        debug_msgs.angular.x = gis;
        debug_msgs.angular.y = e;

        if(isRadiale)
        {
        	debug_msgs.angular.z = 1;
        } else {
        	debug_msgs.angular.z = 0;
        }


        debug_pub.publish(debug_msgs);


        if((dist < dist_max && ros::Time::now().toSec()-compt > 2.0) || init != true)
        {
            if (next_goal_client.call(next_goal_msg))
            {
                if( (int)sizeof(next_goal_msg.response.latitude) != 0 )
                {
                    isRadiale           = next_goal_msg.response.isRadiale;
                    latitude_target     = next_goal_msg.response.latitude[0];
                    longitude_target    = next_goal_msg.response.longitude[0];
                    still_n_mission     = next_goal_msg.response.remainingMissions;
                    if(isRadiale)
                    {
                        yaw_radiale = atan2(longitude_target - next_goal_msg.response.longitude[1], latitude_target - next_goal_msg.response.latitude[1]);
                    }
                } else {
                    state = "IDLE";
                }
                init = true;
            } else{
                ROS_ERROR("Failed to call service");
            }
            compt = ros::Time::now().toSec();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

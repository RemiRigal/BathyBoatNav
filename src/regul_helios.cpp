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

bool computeDistance()
{
    dist = pow(pow(latitude_target - latitude_boat,2) + pow(longitude_target - longitude_boat,2), 0.5);
    
    return dist;
}
sudo socat -d -d /dev/gps,raw,echo=0,b9600 /dev/sbg_aux,raw,echo=0,b9600

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

    double compt = ros::Time::now().toSec();

        // Ros init

    ros::init(argc, argv, "regul_helios");
    ros::NodeHandle n;

    ros::Rate loop_rate(25);

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
    
        // New GPS client

    ros::ServiceClient next_goal_client = n.serviceClient<BathyBoatNav::next_goal>("/next_goal");
    BathyBoatNav::next_goal next_goal_msg;
    
    while(ros::ok())
    {
        double det;

        computeDistance();
        ROS_INFO("Dist to waypoint : %d\n", dist);

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
            u_yaw = atan(e)/3.0;
        }

        cons_msgs.angular.z = u_yaw;
        cons_msgs.linear.x  = 0;

        cons_pub.publish(cons_msgs);

        if(dist < dist_max && ros::Time::now().toSec()-compt > 2.0)
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
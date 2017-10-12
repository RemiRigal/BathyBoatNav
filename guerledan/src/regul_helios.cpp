#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>

#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"

#include "simu_init/depth.h"

#include "std_srvs/Empty.h"

using namespace std;

double gis;
double target_x, target_y;

double x_boat, y_boat;
double roll, pitch, yaw_boat, yaw_target;

double u_yaw;
double u_vitesse;

string name;
string name_wing;
string regul;

const double Pi = 3.14159265358979323846;

bool checkDistance()
{
    double dist = pow(pow(target_x - x_boat,2) + pow(target_y - y_boat,2), 0.5);
    
    return (dist<3.0) ? true : false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "regul_helios");
    ros::NodeHandle n;

    n.param<string>("Name_boat", name, "unknown");
    n.param<string>("Regul", regul, "unknown");

    u_yaw = 0;
    u_vitesse = 0;
    
    target_x = 0;
    target_y = 0;
    
    double e;

    tf::Quaternion q_boat;
    tf::Quaternion q_target;
    
    ros::Rate loop_rate(25);

        // Tf Listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped listenTarget;
    geometry_msgs::TransformStamped listenBoat;

        // Consigne
    ros::Publisher cons_pub = n.advertise<geometry_msgs::Twist>("cons_boat", 1000);
    geometry_msgs::Twist cons_msgs;
    
        // New GPS client
    ros::ServiceClient new_gps_client = n.serviceClient<std_srvs::Empty>("/new_gps_trigger");
    std_srvs::Empty null_msg;
        
    double compt = ros::Time::now().toSec();
    
    // right_depth = -10;

    while(ros::ok())
    {
            // Coord Boat
        try{
            listenBoat = tfBuffer.lookupTransform("map", name, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        x_boat = listenBoat.transform.translation.x;
        y_boat = listenBoat.transform.translation.y;

        q_boat.setX(listenBoat.transform.rotation.x);
        q_boat.setY(listenBoat.transform.rotation.y);
        q_boat.setZ(listenBoat.transform.rotation.z);
        q_boat.setW(listenBoat.transform.rotation.w);

        tf::Matrix3x3(q_boat).getRPY(roll, pitch, yaw_boat);

            // Coord cible
        try{
            listenTarget = tfBuffer.lookupTransform("map", "target", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        target_x = listenTarget.transform.translation.x;
        target_y = listenTarget.transform.translation.y;

        q_target.setX(listenTarget.transform.rotation.x);
        q_target.setY(listenTarget.transform.rotation.y);
        q_target.setZ(listenTarget.transform.rotation.z);
        q_target.setW(listenTarget.transform.rotation.w);
    
        tf::Matrix3x3(q_target).getRPY(roll, pitch, yaw_target);

        double det;

        if(regul.compare("yaw") == 0)
        {
            gis = atan2(target_y - y_boat, target_x - x_boat);
        } else if(regul.compare("line") == 0) {
            det     = (x_boat - target_x)*(-sin(yaw_target)) - (-cos(yaw_target))*(y_boat - target_y);
            gis     = (yaw_target - atan(det)/2.0) - yaw_boat;
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

        cons_pub.publish(cons_msgs);
        
        if(checkDistance() && ros::Time::now().toSec()-compt > 2.0)
        {
            if (new_gps_client.call(null_msg))
            {
                ROS_INFO("Message from regul_helios delivered");
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
#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;

double dt = 0.1;

double x[2], yaw, speed;

double u_yaw, u_speed;

string name;

double offset_x, offset_y;

const double Pi = 3.14159265358979323846;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_yaw   = msg->angular.z;
    u_speed = 0;
}

void evol()
{
    x[0]    += speed*dt*cos(yaw);
    x[1]    += speed*dt*sin(yaw);
    
    yaw     += dt*u_yaw;
    speed   += dt*u_speed;
}

int main(int argc, char** argv)
{
    // ROS Init

    ros::init(argc, argv, "simu_boat");
    ros::NodeHandle n;
    
    // Initial parameters

    n.param<double>("Pos_x", x[0], 0);
    n.param<double>("Pos_y", x[1], 0);
    n.param<double>("Yaw", yaw, 0);
    n.param<double>("Speed", speed, 1);

    n.param<string>("Name", name, "unknown");

    ros::Rate loop_rate(25);
    
    tf::Quaternion q;
    
    // Setting offset for RVIZ

    offset_y = x[0];
    offset_y = x[1];

    // Position history
    
    geometry_msgs::Point pt_boat;
    pt_boat.x = x[0];
    pt_boat.y = x[1];
    pt_boat.z = 0;

    vector<geometry_msgs::Point> pos;
    
    // Boat marker
    
    ros::Publisher mark_pub = n.advertise<visualization_msgs::Marker>("/mark_boat", 1000);
    visualization_msgs::Marker msgs_boat;

    // Line marker
    
    ros::Publisher mark_line = n.advertise<visualization_msgs::Marker>("/line_boat", 1000);
    visualization_msgs::Marker msgs_line;

    // Publisher
    
    ros::Publisher rens_pub = n.advertise<geometry_msgs::Twist>("data_boat", 1000);
    geometry_msgs::Twist data_boat;

    // Subscriber
    
    ros::Subscriber cons_sub = n.subscribe("cons_boat", 1000, chatCallback);
    
    // TF
    
    tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.frame_id    = "map";
    transformStamped.child_frame_id     = name;
    transformStamped.transform.translation.z = 0.0;
    
    while(ros::ok())
    {
        evol();
        
        q.setRPY(0, 0, yaw);
        
        // TF evolution and send
        
        transformStamped.header.stamp = ros::Time::now();
        
        transformStamped.transform.translation.x  = x[0] - offset_x;
        transformStamped.transform.translation.y  = x[1] - offset_y;
        transformStamped.transform.rotation.x     = q.getX();
        transformStamped.transform.rotation.y     = q.getY();
        transformStamped.transform.rotation.z     = q.getZ();
        transformStamped.transform.rotation.w     = q.getW();
        
        br.sendTransform(transformStamped);

        // Data boat

        data_boat.linear.x = x[0];
        data_boat.linear.y = x[1];

        data_boat.angular.x = yaw;

        rens_pub.publish(data_boat);

        // Boat marker

        msgs_boat.header.frame_id = name;
        msgs_boat.header.stamp = ros::Time::now();
        msgs_boat.ns = ros::this_node::getNamespace();
        msgs_boat.id = 0;
        // msgs_boat.type = visualization_msgs::Marker::MESH_RESOURCE;
        msgs_boat.type = visualization_msgs::Marker::SPHERE;
        msgs_boat.action = visualization_msgs::Marker::ADD;
        msgs_boat.scale.x = 1.0;
        msgs_boat.scale.y = 1.0;
        msgs_boat.scale.z = 1.0;
        msgs_boat.color.a = 1.0;
        msgs_boat.color.r = 1.0;
        msgs_boat.color.g = 1.0;
        msgs_boat.color.b = 1.0;
        // msgs_boat.mesh_resource = "package://BathyBoatNav/mesh/boat.dae";
        
        mark_pub.publish(msgs_boat);
        
        // Draw line

        pt_boat.x = x[0];
        pt_boat.y = x[1];
        pt_boat.z = 0;
        
        pos.push_back(pt_boat);
        
        // Line marker
        
        msgs_line.header.frame_id = "map";
        msgs_line.header.stamp = ros::Time::now();
        msgs_line.ns = ros::this_node::getNamespace();
        msgs_line.id = 0;
        msgs_line.type = visualization_msgs::Marker::LINE_STRIP;
        msgs_line.action = visualization_msgs::Marker::ADD;
        msgs_line.points = pos;
        msgs_line.scale.x = 0.2;
        msgs_line.color.a = 0.5;
        msgs_line.color.r = 0.0;
        msgs_line.color.g = 1.0;
        msgs_line.color.b = 0.0;

        mark_line.publish(msgs_line);

        ros::spinOnce();
        
        loop_rate.sleep();
    }

    return 0;
}

#include "ros/ros.h"
#include <cmath>
#include <string>
#include <vector>

#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std;

double x[3];
double dt = 0.1;
double u_yaw;
double speed;

string name;

const double Pi = 3.14159265358979323846;

void chatCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_yaw   = msg->angular.z;
}

void evol()
{
    x[0] = x[0] + speed*dt*cos(x[2]);
    x[1] = x[1] + speed*dt*sin(x[2]);
    x[2] = x[2] + dt*u_yaw;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simu_boat");
    ros::NodeHandle n;
    
        // Parametres initiaux
    n.param<double>("Pos_x", x[0], 0);
    n.param<double>("Pos_y", x[1], 0);
    n.param<double>("Theta", x[2], 0);
    n.param<double>("Cons_u", u_yaw, 0);
    n.param<double>("Vitesse_boat", speed, 1);
    n.param<string>("Name_boat", name, "unknown");
    
    ros::Rate loop_rate(25);
    
    tf::Quaternion q;
    
        // Liste de points
    geometry_msgs::Point pt_boat;
    geometry_msgs::Point pt_magneto;
    pt_boat.x = x[0];
    pt_boat.y = x[1];
    pt_boat.z = 0;

    vector<geometry_msgs::Point> pos;
    
        // Marker du bateau
    ros::Publisher mark_pub = n.advertise<visualization_msgs::Marker>("/mark_boat", 1000);
    visualization_msgs::Marker msgs_boat;

        // Marker de ligne
    ros::Publisher mark_line = n.advertise<visualization_msgs::Marker>("/line_boat", 1000);
    visualization_msgs::Marker msgs_line;

        // Pub data boat
    ros::Publisher rens_pub = n.advertise<geometry_msgs::Twist>("data_boat", 1000);
    geometry_msgs::Twist data_boat;

        // Message contenant la consigne
    ros::Subscriber cons_sub = n.subscribe("cons_boat", 1000, chatCallback);
    
        // TF
    tf2_ros::TransformBroadcaster br;
    
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.frame_id    = "map";
    transformStamped.child_frame_id     = name;
    transformStamped.transform.translation.z      = 0.0;
    
    while(ros::ok())
    {
        evol();
        
        q.setRPY(0, 0, x[2]);
        
            // Topic tf
        transformStamped.header.stamp = ros::Time::now();
        
        transformStamped.transform.translation.x  = x[0];
        transformStamped.transform.translation.y  = x[1];
        transformStamped.transform.rotation.x     = q.getX();
        transformStamped.transform.rotation.y     = q.getY();
        transformStamped.transform.rotation.z     = q.getZ();
        transformStamped.transform.rotation.w     = q.getW();
        
        br.sendTransform(transformStamped);

            // Data boat
        //data_boat.linear.x = speed;
        data_boat.linear.x = x[0];
        data_boat.linear.y = x[1];

        data_boat.angular.x = x[2];
        data_boat.angular.y = 0.0;
        data_boat.angular.z = 0.0;

        rens_pub.publish(data_boat);
        
                // Topic pour RVIZ

            // Mark boat
        msgs_boat.header.frame_id = name;
        msgs_boat.header.stamp = ros::Time::now();
        msgs_boat.ns = ros::this_node::getNamespace();
        msgs_boat.id = 0;
        msgs_boat.type = visualization_msgs::Marker::MESH_RESOURCE;
        msgs_boat.action = visualization_msgs::Marker::ADD;
        msgs_boat.scale.x = 1.0;
        msgs_boat.scale.y = 1.0;
        msgs_boat.scale.z = 1.0;
        msgs_boat.color.a = 1.0;
        msgs_boat.color.r = 1.0;
        msgs_boat.color.g = 1.0;
        msgs_boat.color.b = 1.0;
        msgs_boat.mesh_resource = "package://BathyBoatNav/mesh/boat.dae";
        
        mark_pub.publish(msgs_boat);
        
            // Draw line
        pt_boat.x = x[0];
        pt_boat.y = x[1];
        pt_boat.z = 0;
        
        pos.push_back(pt_boat);
        
                // Mark Line
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

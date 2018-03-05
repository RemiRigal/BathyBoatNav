#include <iostream>
#include <string>
#include <cmath>

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "BathyBoatNav/message.h"
#include "BathyBoatNav/next_goal.h"
#include "BathyBoatNav/new_state.h"
#include "BathyBoatNav/offset_simu.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include "../include/state.h"

using namespace std;

double yaw_bar, speed_bar;

double P, I;

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

State state;

void computeDistance()
{
    dist = pow(pow(x_target - x_boat,2) + pow(y_target - y_boat,2), 0.5);
}

void posCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    x_boat = msg->linear.x;
    y_boat = msg->linear.y;
    yaw_boat = msg->angular.z;
}

void speedCallback(const std_msgs::Float64::ConstPtr& msg)
{
    speed_bar = msg->data;
}

bool stateCallback(BathyBoatNav::new_state::Request &req, BathyBoatNav::new_state::Response &res)
{
    int idx_state = req.state;

    if( idx_state <= 4 && idx_state >= 0 )
    {
        state = State(idx_state);
        res.success = true;
    } else {
        res.success = false;
    }

    return true;
}

int main(int argc, char** argv)
{
    int num_waypoints = 1;
    double e;
    double dead_zone;
    double full_left;
    double det = 0.0;
    double k_I, k_P;
    double dist_line = 0.0;

    bool init = true;
    bool isSimulation;

    u_yaw = 0;
    u_vitesse = 0;

    I = 0;

    // Ros init

    ros::init(argc, argv, "regul_helios");
    ros::NodeHandle n;

    ros::Rate loop_rate(25);

    // Initials parameters

    n.param<double>("Accept_dist", dist_max, 10.0);
    n.param<double>("P", k_P, 0.5);
    n.param<double>("I", k_I, 0.5);
    n.param<double>("Dead_zone", dead_zone, 0.05);
    n.param<double>("Full_left", full_left, 2.5);
    n.param<bool>("Simu", isSimulation, true);

    // Subscriber

    ros::Subscriber data_sub   = n.subscribe("gps_angle_boat",   1000, posCallback);
    ros::Subscriber speed_sub   = n.subscribe("/speed_hat",   1000, speedCallback);

    // Publisher

    ros::Publisher cons_pub = n.advertise<geometry_msgs::Twist>("cons_boat", 1000);
    geometry_msgs::Twist cons_msgs;

    ros::Publisher debug_pub = n.advertise<geometry_msgs::Twist>("debug_boat", 1000);
    geometry_msgs::Twist debug_msgs;

    // Next goal service

    ros::ServiceClient next_goal_client = n.serviceClient<BathyBoatNav::next_goal>("next_goal");
    BathyBoatNav::next_goal next_goal_msg;

    
    ros::ServiceClient offset_client = n.serviceClient<BathyBoatNav::offset_simu>("offset_position");
    BathyBoatNav::offset_simu offset_msg;

    ros::ServiceClient change_state_client = n.serviceClient<BathyBoatNav::message>("/changeStateSrv");
    BathyBoatNav::message change_state_msg;

    ros::ServiceServer state_srv = n.advertiseService("/regul_state", stateCallback);

    while(ros::ok())
    {
        if(state == RUNNING)
        {
            computeDistance();

            if(dist < dist_max || init)
            {
                if (next_goal_client.call(next_goal_msg))
                {
                    if( (int)sizeof(next_goal_msg.response.latitude) != 0 )
                    {
                        isRadiale       = next_goal_msg.response.isRadiale;
                        x_target        = next_goal_msg.response.latitude[0];
                        y_target        = next_goal_msg.response.longitude[0];
                        still_n_mission = next_goal_msg.response.remainingMissions;
                        if(isRadiale)
                        {
                            x_target_start_line = next_goal_msg.response.latitude[1];
                            y_target_start_line = next_goal_msg.response.longitude[1];
                            
                            yaw_radiale = atan2(x_target - x_target_start_line, y_target - y_target_start_line);
                        }
                    } else {
                        change_state_msg.request.message = "IDLE";
            
                        if(change_state_client.call(change_state_msg))
                        {                
                            if(change_state_msg.response.success)
                            {
                                ROS_INFO("State changed");
                            } else {
                                ROS_WARN("Failed to change state to IDLE.");
                            }
                        } else {
                            ROS_WARN("Call to fsm failed");
                        }
                    }
                    
                    if(init && isSimulation)
                    {
                        if(isRadiale)
                        {
                            offset_msg.request.x_lambert = x_target_start_line;
                            offset_msg.request.y_lambert = y_target_start_line;
                        } else {
                            offset_msg.request.x_lambert = x_target;
                            offset_msg.request.y_lambert = y_target;
                        }
                        
                        if (offset_client.call(offset_msg))
                        {
                            ROS_INFO("Offset given");
                        } else{
                            ROS_ERROR("Failed to call simu node");
                        }
                    }

                    init = false;
                    num_waypoints ++;
                } else{
                    ROS_ERROR("Failed to call service");
                }
            }

            if(isRadiale)
            {
                det         = (x_target - x_target_start_line)*(y_boat - y_target_start_line) - (x_boat - x_target_start_line)*(y_target - y_target_start_line);
                dist_line   = det / (pow(pow(x_target - x_target_start_line,2) + pow(y_target - y_target_start_line,2), 0.5));
                yaw_bar     = yaw_radiale + 0.4 * tanh(dist_line);
            } else {
                yaw_bar     = atan2(x_target - x_boat, y_target - y_boat);
            }

            e   = 2.0*atan(tan((yaw_bar - yaw_boat)/2.0));
            if(abs(e) < dead_zone)
            {
                u_yaw = 0.0;
            } else if (abs(e) > full_left) {
                u_yaw = 0.8;
            } else {
                P = k_P*e;
                I += k_I*e;
                u_yaw = abs(P + I) >= 1 ? 1 : (P + I);
            }

            cons_msgs.angular.z = u_yaw;
            cons_msgs.linear.x  = 0.7;

            cons_pub.publish(cons_msgs);

            debug_msgs.linear.x = isRadiale;
            debug_msgs.linear.y = yaw_radiale;
            debug_msgs.linear.z = dist_line;
            debug_msgs.angular.x = yaw_bar;
            debug_msgs.angular.y = e;
            debug_msgs.angular.z = num_waypoints;

            debug_pub.publish(debug_msgs);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

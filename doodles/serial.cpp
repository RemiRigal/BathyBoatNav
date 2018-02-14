#include <ros/ros.h>
#include <string>
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

using namespace std;

serial::Serial ser;

int main (int argc, char** argv)
{
    string path;
    string data;

    int baud;

        // Ros init

    ros::init(argc, argv, "serial");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(10);
    
        // Initials parameters
    
    n.param<string>("Path",     path, "/dev/ttyUSB0");
    n.param<int>("Baudrate",    baud, 4800);


    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = n.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort(path);
        ser.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        exit(1);
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        ROS_ERROR_STREAM("Serial not opened");
        exit(1);
    }

    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            data = ser.read(ser.available());
            cout << data << endl;
        }

        loop_rate.sleep();

    }
}
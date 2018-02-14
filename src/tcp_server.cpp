#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <sys/time.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

using namespace std;

int socket_RV;
int socket_service;
struct sockaddr_in adr;
socklen_t lgadresse;

string msg;

bool isSending;
int port;

double x[2], yaw, pitch, roll, vit, wifi_lvl;
double u_yaw;

// SIGACTION
void signals_handler(int signal_number)
{
	printf("\nKilling server\n");
	close(socket_RV);
	close(socket_service);
	fflush(stdout);
	printf("\nClosed cleanly\n");
	exit(1);
}

void send_to ()
{
	ros::Rate loop_rate(10);
	char buffer[500];
	while(ros::ok())
	{
		sprintf(buffer,"$POS;%lf;%lf;%lf;%lf;%lf;%lf;%lf;%lf\n",ros::Time::now().toSec(), x[0], x[1], yaw, roll, pitch, 0.0, 100.0);

		if( send(socket_service, buffer, strlen(buffer), 0) < 0 )
		{
			printf("End of connection\n");
			break;
		}

		printf("Sent : %s\n", buffer);

		ros::spinOnce();
        loop_rate.sleep();
	}
}

void rec_from ()
{
	ros::Rate loop_rate(10);
	char c[1];
	string rcv_msg;
	int res;
	int i, j;

	while(ros::ok())
	{
		do{
			res = recv(socket_service, c, 1, 0);

			if( res <= 0 )
			{
				printf("End of connection\n");
				break;
			}
			
			rcv_msg.append(c);
		} while((int)c[0] != 0);

		printf("Recv : %s\n", rcv_msg.c_str());
		rcv_msg.clear();
	}
}

void accept_loop()
{
	while(ros::ok())
	{

		printf("Waiting for connection...\n");
		socket_service = accept(socket_RV,(struct sockaddr *)&adr, &lgadresse);
		printf("Connection successful\n");

		if (socket_service < 0)
		{
	        perror("Accept error");
        	exit(1);
		}

		if(isSending)
		{
			send_to();
		} else {
			rec_from();
		}
		
	}
}

void server(int port)
{
	

	if((socket_RV=socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		perror("Unable to create socket");
		exit(1);
	}

	adr.sin_family 		= AF_INET;
	adr.sin_port 		= htons(port);
	adr.sin_addr.s_addr = htonl(INADDR_ANY);

	int opt = 1;
	setsockopt(socket_RV,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(int));

	if (bind(socket_RV, (struct sockaddr *) &adr, sizeof(adr))==-1)
	{
		perror("No bind");
		exit(1);
	}
	
	if (listen(socket_RV,1)==-1) // Ecoute si quelqu'un se connecte
	{
		perror("Unable to listen");
		exit(1);
	}
	
	accept_loop();
}

/*
void dataCallback(const std_msgs::String::ConstPtr& ros_msg)
{
	msg = ros_msg->data;
}
*/

void dataCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    x[0] 	= msg->linear.x;
    x[1] 	= msg->linear.y;

    yaw 	= msg->angular.x;
    roll 	= msg->angular.y;
    pitch 	= msg->angular.z;
}

	// Main

int main(int argc, char *argv [])
{
	// SIGACTION
    struct sigaction action;
    action.sa_handler = signals_handler;
    sigemptyset(& (action.sa_mask));
    sigaction(SIGKILL, 	& action, NULL);
    sigaction(SIGTERM, 	& action, NULL);
    sigaction(SIGQUIT, 	& action, NULL);
    sigaction(SIGINT, 	& action, NULL);
    // END SIGACTION

	ros::init(argc, argv, "tcp_serveur");
    ros::NodeHandle n;
    
    //n.param<bool>("isSending", isSending, true);
    //n.param<int>("port", port, 29200);

    isSending = argv[2];
    port = atoi(argv[1]);

    cout << "Type of server : " << isSending << " | Port : " << port << endl;

	// Subscribe msgs
    //ros::Subscriber status_sub = n.subscribe("/msg_tcp", 1000, dataCallback);

    ros::Subscriber status_sub = n.subscribe("data_boat", 1000, dataCallback);


	printf("Starting server\n");

	server(port);

	printf("Killing server\n");
	close(socket_RV);
	close(socket_service);

	return EXIT_SUCCESS;
}
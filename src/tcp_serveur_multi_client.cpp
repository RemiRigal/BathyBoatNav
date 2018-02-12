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

#include "std_msgs/String.h"

using namespace std;

int socket_RV;
int socket_service;
struct sockaddr_in adr;
socklen_t lgadresse;

string msg;
int pid;
int ppid;

fd_set fdset;
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

void send_process (int sock)
{
	ros::Rate loop_rate(10);

	char buffer[500];
	while(ros::ok())
	{
		if(ppid != getppid())
		{
			printf("Parent killed\n");
			close(socket_service);
			exit(1);
		}

		sprintf(buffer, "%s", msg);

		if( send(socket_service, buffer, strlen(buffer), 0) <= 0 )
		{
			break;
		}

		ros::spinOnce();
        loop_rate.sleep();
	}
}

void accept_connection()
{
	while(1)
	{
		struct timeval tv = {2, 2};
		FD_ZERO(&fdset);
		FD_SET(socket_RV, &fdset);
		select(socket_RV+1, &fdset, NULL, NULL, &tv);

		//printf("%d\n", FD_ISSET(socket_RV, &fdset));
		if (FD_ISSET(socket_RV, &fdset))
		{
			printf("Connection happening\n");
			socket_service = accept(socket_RV,(struct sockaddr *)&adr, &lgadresse); // Accepte la connection
			printf("Connection successful\n");

			if (socket_service < 0)
			{
		        perror("Accept error");
	        	exit(1);
			}

			if ((pid = fork()) < 0)
			{
				perror("ERROR on fork");
				exit(1);
			}

			if (pid == 0)
			{
				close(socket_RV);
				send_process(socket_service);
				close(socket_service);
				printf("Client disconnected\n");
				exit(0);
			} else {
				printf("PID fils : %d\n", pid);
				close(socket_service);
			}
		}
	}
}

int serveur(int port)
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
	
	accept_connection();

    return socket_RV;
}

void dataCallback(const std_msgs::String::ConstPtr& ros_msg)
{
	msg = ros_msg->data;
	cout << "Msg -> " << msg << endl;
}

	// Main

int main(int argc, char *argv [])
{
	ppid = getpid();

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
    
	// Subscribe msgs
    ros::Subscriber status_sub = n.subscribe("/msg_tcp", 1000, dataCallback);

	printf("Starting server\n");
	int socket_RV 	= serveur(29200);

	printf("Killing server\n");
	close(socket_RV);

	return EXIT_SUCCESS;
}
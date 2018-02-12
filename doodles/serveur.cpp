#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>

int socket_RV;
int socket_service;
struct sockaddr_in adr;
socklen_t lgadresse;

double x[2], yaw, pitch, roll, vit, wifi_lvl;
double u_yaw;

// SIGACTION
void signals_handler(int signal_number)
{
	close(socket_RV);
	fflush(stdout);
	printf("\nClosed cleanly\n");
	exit(1);
}

void accept_connection()
{
	while(1)
	{
		printf("Waiting for client...\n");
		socket_service = accept(socket_RV,(struct sockaddr *)&adr, &lgadresse); // Accepte la connection
		printf("Connection successful\n");

		if (socket_service < 0)
		{
	        perror("Accept error");
        	exit(1);
		}

		pid = fork();

		if (pid < 0) {
			perror("ERROR on fork");
			exit(1);
		}

		if (pid == 0) {
			close(socket_RV);
			doprocessing(socket_service);
			exit(0);
		} else {
			close(socket_service);
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
	
	accept_connection(socket_RV);


    return socket_RV;
}

void speak(char* msg, int socket_service)
{
	char c;    
	int i = 0;
	while(msg[i] != 0)
	{
		c = msg[i];
		write(socket_service, &c, 1); // On envoie caractere par caractere.
		i++;
	}
}

void dataCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    x[0] 	= msg->linear.x;
    x[1] 	= msg->linear.y;

    yaw 	= msg->angular.x;
    roll 	= msg->angular.y;
    pitch 	= msg->angular.z;
}

void consCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    u_yaw 	= msg->angular.z;
}

	// Main

int main(int argc, char *argv [])
{
	ros::init(argc, argv, "serveur");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(10);

	int compteur = 0;
	char buffer[100];

	// Subscribe msgs
    ros::Subscriber status_sub = n.subscribe("data_boat", 1000, dataCallback);

	printf("Starting server\n");
	int socket_RV 	= serveur();
	
	while(ros::ok())
	{
		//send(socket_service, argv[3], strlen(argv[3]), 0);
		sprintf(buffer,"$POS;%lf;%lf;%lf;%lf;%lf;%lf;%lf;%lf\n",ros::Time::now().toSec(), x[0], x[1], yaw, roll, pitch, 0.0, 100.0);

		speak(buffer, socket_RV);

		sprintf(buffer,"$MOT;%lf;%lf;%lf\n",ros::Time::now().toSec(), u_yaw*100, -u_yaw*100);

		speak(buffer, socket_RV);

		ros::spinOnce();
        loop_rate.sleep();
	}

	close(socket_RV);

	exit(1);
}
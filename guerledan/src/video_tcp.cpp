#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

using namespace std;
using namespace cv;

double x[2], yaw, pitch, roll, vit, wifi_lvl;
Mat cam;

	// Video capture


	//  Initialisation du serveur

/* Fonction serveur:
Initialise le serveur
Return
	int 		-> ID du socket
*/
int serveur()
{
	char nom[30];
	int socket_RV;
	int socket_service;
	struct sockaddr_in adr;
	socklen_t lgadresse;
	
	int option = 1;
/*
	création du socket
*/

	if((socket_RV=socket(AF_INET, SOCK_STREAM, 0)) == -1) // Cree le socket
		{
		perror("Unable to create socket");
		exit(1);
		}

	setsockopt(socket_RV, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

	adr.sin_family 		= AF_INET;
	adr.sin_port 		= htons(29202);
	adr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(socket_RV, (struct sockaddr *) &adr, sizeof(adr))==-1) // Bind du socket avec l'adresse
		{
		perror("No bind");
		exit(1);
		}

	if (listen(socket_RV,1)==-1) // Ecoute si quelqu'un se connecte
		{
		perror("Unable to listen");
		exit(1);
		}
	
	printf("Waiting for client...\n");
	//socket_service = accept(socket_RV,(struct sockaddr *)&adr, &lgadresse); // Accepte la connection
	printf("Connection successful\n");

    	return socket_service;
}
	//  Fonction de dialogue

/* Fonction speak:
Envoie une chaine de caractere dans un socket
Arg:
	msg 		-> Chaine de caracteres a envoyer
	socket_RV 	-> ID du socket a utiliser
*/
void speak(char* msg, int socket_RV)
{
	char c;    
	int i = 0;
	while(msg[i] != 0)
	{
		c = msg[i];
		write(socket_RV, &c, 1); // On envoie caractere par caractere.
		i++;
	}
}

	// Main

int main(int argc, char *argv [])
{
	ros::init(argc, argv, "serveur");
    ros::NodeHandle n;
    
        // Parametres initiaux
    //n.param<double>("Pos_x", x[0], 0);
    
    ros::Rate loop_rate(10);

	printf("Starting server\n");
	int socket_RV 	= serveur();

	//char msg[25];

	int compteur = 0;
	char buffer[100];

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat frame;

	while(ros::ok())
	{
		// , mat->data.ptr
		cap >> frame;
		ROS_INFO("Info -> %d \n", frame.rows*frame.cols, );
		//speak(buffer, socket_RV);

		ros::spinOnce();
        loop_rate.sleep();
	}

	close(socket_RV);

	exit(1);
}
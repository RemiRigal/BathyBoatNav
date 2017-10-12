#include "ros/ros.h"
#include <serial/serial.h>
#include <string>

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
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>

using namespace std;

serial::Serial ser;

int serialInit()
{
	try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(4800);
        //serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        //ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        printf("Serial Port initialized");
    }else{
        return -1;
    }
}

	//  Initialisation du serveur

/* Fonction client:
Initialise le serveur
Return
	int 		-> ID du socket
*/
int client()
{
    int socket_RV;
	struct hostent *hote;
	struct sockaddr_in adr;
	
	printf("Creating Socket... ");

	if((socket_RV=socket(AF_INET, SOCK_STREAM, 0)) == -1) // Cree un socket
	{
		perror("socket rendez-vous");
		exit(1);
	}

	printf("OK\n");

	hote = gethostbyname("192.168.0.32");
	adr.sin_family=AF_INET;
	adr.sin_port=htons(5017);
	bcopy(hote->h_addr, &adr.sin_addr.s_addr, hote->h_length);

	printf("Waiting for connection... ");

	if (connect(socket_RV,(struct sockaddr *)&adr, sizeof(adr))==-1) // Se connecte
	{
		perror("connect");
		exit(1);
	}

	printf("OK\n");

    return socket_RV;
}

	//  Fonction de dialogue

/* Fonction hear:
Affiche les informations lues
Arg:
	socket_RV 	-> ID du socket a utiliser
*/
void hear(int socket_RV)
{
	char msg;
	char str[1000];
	int compteur = 0;
	msg          = EOF;

	string data;

	for(;;)    
	{
		compteur = read(socket_RV, &msg, 1);
		printf("%d\t-> %c\t| %d\n", compteur, msg, (int)msg); // print de debuggage
        if(ser.available())
        {
            data = ser.read(ser.available());
            cout << data << endl;
        } else {
        	cout << "Serial not available" << endl;
        }
	}
}

	// Main

int main(int argc, char *argv [])
{
	ros::init(argc, argv, "serveur");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(10);

	printf("Starting client\n");
	int socket_RV 	= client();

	printf("Init Serial Connection\n");
	serialInit();

	while(ros::ok())
	{
		ros::spinOnce();
        loop_rate.sleep();
        hear(socket_RV);
	}

	close(socket_RV);

	exit(1);
}
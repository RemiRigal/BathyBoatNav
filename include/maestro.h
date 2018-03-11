// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
 
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

/**
 * Gets the position of a Maestro channel.
 * See the "Serial Servo Commands" section of the user's guide.
 * @param fd
 * @param channel
 * @return
 */
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
 
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }
 
  return response[0] + 256*response[1];
}

/**
 * Sets the target of a Maestro channel.
 * See the "Serial Servo Commands" section of the user's guide.
 * The units of 'target' are quarter-microseconds.
 * @param fd
 * @param channel
 * @param target
 * @return
 */
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
    unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }
    return 0;
}

/**
 * Connexion de la maestro
 * @param device
 * @return
 */
int maestroConnect(const char * device){
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        printf("Connection failed\n");
        perror(device);
        return 1;
    }
    return fd;
}



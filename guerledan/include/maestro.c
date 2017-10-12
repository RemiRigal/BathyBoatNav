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

#include "maestro.h"

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

int maestroSetSpeed(int fd, unsigned char channel, unsigned short target)
{
    unsigned char command[] = {0x87, channel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }
    return 0;
}

int maestroSetAccel(int fd, unsigned char channel, unsigned short target)
{
    unsigned char command[] = {0x89, channel, target & 0x7F, target >> 7 & 0x7F};
    if (write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }
    return 0;
}

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

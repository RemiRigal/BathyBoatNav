#ifndef _STATE_H
#define _STATE_H

#include <stdlib.h>

enum State{
    IDLE 		= 0,
    RUNNING 	= 1,
    PAUSE 		= 2,
    RTL 		= 3,
    EMERGENCY 	= 4
};

#endif
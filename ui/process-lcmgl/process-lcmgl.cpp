/*
 * Displays process and timesync information on LCMGL
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>


using namespace std;


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/lcmt_process_control.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
   

lcm_t * lcm;

double obstacleXY1[2], obstacleXY2[2], obstacleHeight, obstacleBottom;

bot_lcmgl_t* lcmgl;

// bot frames global
BotFrames *botFrames;

lcmt_process_control_subscription_t *process_sub;


struct StringsOutStruct {
    string time;
    string logfilesize;
};

StringsOutStruct stringsOut;



static void usage(void)
{
        fprintf(stderr, "usage: process-lcmgl process-control-channel lcmgl-channel-name\n");
        fprintf(stderr, "    process-control-channel: LCM channel name with process control messages\n");
        fprintf(stderr, "    lcmgl-channel-name : LCM channel to publish LCMGL messages on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./process-lcmgl process_control process_lcmgl\n");
        fprintf(stderr, "    reads optotrak LCM messages and draws the position using  LCMGL\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_process_control_unsubscribe(lcm, process_sub);
    lcm_destroy (lcm);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

int temp = 0;
void process_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_process_control *msg, void *user)
{

    temp ++;
    stringsOut.time = to_string(temp);
    
}



int main(int argc,char** argv)
{
    
    char *channelProcess = NULL;
    char *channelLcmGl = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    channelProcess = argv[1];
    channelLcmGl = argv[2];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    

    signal(SIGINT,sighandler);
    
    lcmgl = bot_lcmgl_init(lcm, channelLcmGl);

    
    process_sub = lcmt_process_control_subscribe(lcm, channelProcess, &process_handler, NULL);
    
    printf("Receiving:\nProcess control: %s\nPublishing LCM:\n\tLCMGL: %s\n", channelProcess, channelLcmGl);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
        
        // everytime we do an lcm handle, refresh the display
        printf("\rPlane time: %s\nLogfile size: %s", stringsOut.time.c_str(), stringsOut.logfilesize.c_str());
    }

    return 0;
}

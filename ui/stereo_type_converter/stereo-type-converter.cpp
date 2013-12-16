/*
 * Converts old stereo type to new stereo type.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <mutex>


#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_old2.h"


using namespace std;

int g_v_number = 0;
int last_frame_number = 0;

lcm_t * lcm;

lcmt_stereo_old2_subscription_t * stereo_old2_sub;

char *channelStereo = NULL;

int numFrames = 0;


static void usage(void)
{
        fprintf(stderr, "usage: stereo-type-converter stereo-old-channel stereo-new-channel\n");
        fprintf(stderr, "    ./stereo-type-converter stereo-old stereo\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_stereo_old2_unsubscribe(lcm, stereo_old2_sub);
    
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


void stereo_old2_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_old2 *msg, void *user)
{
    // republish messages
    // make a new message
    lcmt_stereo newMsg;
    
    newMsg.timestamp = msg->timestamp;
    newMsg.number_of_points = msg->number_of_points;
    
    newMsg.x = msg->x;
    newMsg.y = msg->y;
    newMsg.z = msg->z;
    
    newMsg.grey = msg->grey;
    
    newMsg.frame_number = msg->frame_number;
    
    // figure out the video number
    if (msg->frame_number < last_frame_number) {
        g_v_number ++;
    }
    
    last_frame_number = msg->frame_number;
    newMsg.video_number = g_v_number;
    
    // send the message
    lcmt_stereo_publish(lcm, channelStereo, &newMsg);
    
    // update frame number
    numFrames ++;
}


int main(int argc,char** argv)
{
    char *channelStereoOld = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    channelStereoOld = argv[1];
    channelStereo = argv[2];
    
    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    stereo_old2_sub = lcmt_stereo_old2_subscribe (lcm, channelStereoOld, &stereo_old2_handler, NULL);

    signal(SIGINT,sighandler);

    printf("Reading:\nStereo old: %s\nWriting:\n\tStereo: %s\n", channelStereoOld, channelStereo);
    
    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

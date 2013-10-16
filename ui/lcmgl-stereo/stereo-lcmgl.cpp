/*
 * Draws stereo hits data to LCMGL
 *
 * Author: Amruth Venkatraman, <amruthv@mit.edu> 2013
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

#include "../../LCM/lcmt_optotrak_quat.h"
#include "../../LCM/lcmt_stereo.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
   

lcm_t * lcm;


bot_lcmgl_t* lcmgl;

lcmt_stereo_subscription_t *stereo_sub;

//Global bot frames
BotFrames *botFrames;

static void usage(void)
{
        fprintf(stderr, "usage: optotrak-lcmgl optotrak-quat-channel-name lcmgl-channel-name\n");
        fprintf(stderr, "    optotrak-quat-channel-name : LCM channel name with optotrak QUAT messages\n");
        fprintf(stderr, "    lcmgl-channel-name : LCM channel to publish LCMGL messages on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./optotrak-lcmgl wingeron_x_quat wingeron_x_lcmgl\n");
        fprintf(stderr, "    reads optotrak LCM messages and draws the position using  LCMGL\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
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


void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_point_size(lcmgl, 10.5f);
    
    BotTrans toOpenCv;
    bot_frames_get_trans(botFrames, "opencvFrame", "local", &toOpenCv);
    int numHits = msg -> number_of_points;
    //printf("numHits: %d \n", numHits);
    for (int i=0; i< numHits; i++) {
    
        double originalCoords[3];
        //Get coordinate of the location of the obstacle
        originalCoords[0] = msg -> x[i];
        originalCoords[1] = msg -> y[i];
        originalCoords[2] = -msg -> z[i];
        

        double transPoint[3];
        bot_trans_apply_vec(&toOpenCv,originalCoords, transPoint); 
        //Draw the point
        //printf("x: %f, y: %f, z: %f \n", transPoint[0], transPoint[1], transPoint[2]);
        bot_lcmgl_begin(lcmgl, GL_POINTS);
        bot_lcmgl_color3f(lcmgl,0,0,255);
        bot_lcmgl_vertex3f(lcmgl, transPoint[0], transPoint[1], transPoint[2]);
        bot_lcmgl_end(lcmgl);
    }
    
    //Finish writing to LCMGL
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
    
}



int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *channelLcmGl = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    channelStereo = argv[1];
    channelLcmGl = argv[2];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    

    signal(SIGINT,sighandler);
    
    lcmgl = bot_lcmgl_init(lcm, channelLcmGl);
    
    stereo_sub = lcmt_stereo_subscribe(lcm, channelStereo, &stereo_handler, NULL);
    
    //init frames
    BotParam *param = bot_param_new_from_server(lcm, 0);
    botFrames = bot_frames_new(lcm, param);
    
    printf("Receiving:\nStereo LCM: %s\nPublishing LCM:\n\tLCMGL: %s\n", channelStereo, channelLcmGl);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

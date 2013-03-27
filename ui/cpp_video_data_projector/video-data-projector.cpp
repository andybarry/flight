/*
 * Projects trajectories and stereo collections points onto recorded (or live) video
 * when it receives LCM messages (assumes you'll play them back for logging)
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

#include "opencv2/opencv.hpp"

#include "../../LCM/lcmt_stereo.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "mav_ins_t.h" // from Fixie
#include "mav_gps_data_t.h" // from Fixie

#include "mavconn.h" // from mavconn

using namespace std;
using namespace cv;using namespace std;
using namespace cv;

#define GRAVITY_MSS 9.80665f // this matches the ArduPilot definition

lcm_t * lcm;

lcmt_stereo_subscription_t * stereo_sub;

VideoCapture videoFileCap;


static void usage(void)
{
        fprintf(stderr, "usage: video-data-projector video-file stereo-lcm-channel-name\n");
        fprintf(stderr, "    video-file: .avi file with video\n");
        fprintf(stderr, "    stereo-lcm-channel-name : LCM channel to receive stereo matches on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    video-data-projector videoL-2013-03-27-18-53-43.avi stereo\n");
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

int EightBitToServoCmd(int charInputIn)
{
    return 200/51 * charInputIn + 1000;
}

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
    // if we got a stereo message, grab the next frame of video
    Mat thisImg;
    
    videoFileCap >> thisImg;
    
    imshow("Data on video", thisImg);
}


int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *videoFile = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    videoFile = argv[1];
    channelStereo = argv[2];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    stereo_sub = lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);

    signal(SIGINT,sighandler);

    printf("Reading:\n\tVideo file: %s\n\tStereo LCM: %s\n", videoFile, channelStereo);
    
    // open the video file
    if (videoFileCap.open(videoFile) == true)
    {
        cout << "File opened successfully." << endl;
    } else {
        cout << "ERROR: failed to open video file: " << string(videoFile) << endl;
        exit(1);
    }
    
    namedWindow("Data on video", CV_WINDOW_AUTOSIZE);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

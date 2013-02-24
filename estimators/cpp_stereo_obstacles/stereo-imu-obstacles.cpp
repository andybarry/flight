/*
 * Integrates the 3d stereo data (from LCM) and the IMU data (from LCM)
 * and outputs and obstacle map (tbd)
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

#include "../../LCM/lcmt_gps.h"
#include "../../LCM/lcmt_attitude.h"
#include "../../LCM/lcmt_baro_airspeed.h"
#include "../../LCM/lcmt_stereo.h"
    
lcm_t * lcm;
char *lcm_out = NULL;

lcmt_stereo_subscription_t * stereo_sub;
lcmt_attitude_subscription_t * attitude_sub;
lcmt_baro_airspeed_subscription_t * baro_airspeed_sub;
lcmt_gps_subscription_t * gps_sub;

static void usage(void)
{
        fprintf(stderr, "usage: stereo-imu-obstacles stereo-channel-name attitude-channel-name baro/airspeed-channel-name gps-channel-name\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with stereo LCM messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./stereo-imu-obstacles stereo attitude baro-airspeed gps\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    lcmt_attitude_unsubscribe(lcm, attitude_sub);
    lcmt_gps_unsubscribe(lcm, gps_sub);
    lcm_destroy (lcm);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
}

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
    cout << "got stereo message" << endl;
}

void attitude_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_attitude *msg, void *user)
{
    cout << "got attitude message" << endl;
}

void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user)
{
    cout << "got baro/airspeed message" << endl;
}

void gps_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_gps *msg, void *user)
{
    cout << "got gps message" << endl;
}


int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *channelAttitude = NULL;
    char *channelBaroAirspeed = NULL;
    char *channelGps = NULL;

    if (argc!=5) {
        usage();
        exit(0);
    }

    channelStereo = argv[1];
    channelAttitude = argv[2];
    channelBaroAirspeed = argv[3];
    channelGps = argv[4];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    stereo_sub =  lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);
    attitude_sub =  lcmt_attitude_subscribe (lcm, channelAttitude, &attitude_handler, NULL);
    baro_airspeed_sub =  lcmt_baro_airspeed_subscribe (lcm, channelBaroAirspeed, &baro_airspeed_handler, NULL);
    gps_sub =  lcmt_gps_subscribe (lcm, channelGps, &gps_handler, NULL);



    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tStereo: %s\n\tAttitude: %s\n\tBarometric altitude and airspeed: %s\n\tGPS: %s\n", channelStereo, channelAttitude, channelBaroAirspeed, channelGps);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

/*
 * Simple estimator for 3D wind based on airspeed and GPS velocities
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
#include <mutex>
#include <sys/time.h>

#include "../../../Fixie/build/include/lcmtypes/mav_gps_data_t.h"
#include "../../LCM/lcmt_wind_groundspeed.h"
#include "../../LCM/lcmt_baro_airspeed.h"

lcm_t * lcm;

char *channelWind = NULL;

mav_gps_data_t_subscription_t * gpsSub;
lcmt_baro_airspeed_subscription_t * baroAirSub;

// global mutexes
std::mutex gps_mutex;

// globals for ensuring data has arrived
bool gpsFlag = false;

// globals for holding state between messages
mav_gps_data_t *lastGpsMsg;

static void usage(void)
{
        fprintf(stderr, "usage: wind-estimator gps-channel-name baro-airspeed-channel-name wind-groundspeed-channel-name\n");
        fprintf(stderr, "    gps-channel-name : LCM channel to receive GPS messages on\n");
        fprintf(stderr, "    baro-airspeed-channel-name : LCM channel to recieve barometric altitude and airspeed\n");
        fprintf(stderr, "    wind-groundspeed-channel-name : LCM channel to publish airspeed, estimated ground speed, and estimated wind speed\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./wind-estimator gps baro-airspeed wind-groundspeed\n");
        fprintf(stderr, "    reads LCM messages for GPS and airspped and estimates 3D wind.\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    mav_gps_data_t_unsubscribe(lcm, gpsSub);
    lcmt_baro_airspeed_unsubscribe(lcm, baroAirSub);
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

void gps_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user)
{
    gps_mutex.lock();
    lastGpsMsg = mav_gps_data_t_copy(msg);
    gps_mutex.unlock();

    gpsFlag = true;
}

void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user)
{
    // get the gps mutex lock
    gps_mutex.lock();

    // TODO TODO TODO

    // for now, just return the airspeed as the ground speed

    // STUB STUB STUB STUB STUB

    lcmt_wind_groundspeed windMsg;
    windMsg.utime = getTimestampNow();

    windMsg.airspeed = msg->airspeed;
    windMsg.estimatedGroundSpeed = msg->airspeed; // TODO TODO

    windMsg.wind_x = 0;
    windMsg.wind_y = 0;
    windMsg.wind_z = 0;

    lcmt_wind_groundspeed_publish (lcm, channelWind, &windMsg);

    // unlock
    gps_mutex.unlock();
}


int main(int argc,char** argv)
{
    char *channelBaroAirspeed = NULL;
    char *channelGps = NULL;

    if (argc!=4) {
        usage();
        exit(0);
    }

    channelGps = argv[1];
    channelBaroAirspeed = argv[2];
    channelWind = argv[3];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    gpsSub = mav_gps_data_t_subscribe (lcm, channelGps, &gps_handler, NULL);
    baroAirSub = lcmt_baro_airspeed_subscribe (lcm, channelBaroAirspeed, &baro_airspeed_handler, NULL);


    signal(SIGINT,sighandler);

    printf("Receiving:\n\tGPS LCM: %s\n\tBarometric altitude and airspeed: %s\nPublishing:\n\tWind estimates: %s\n", channelGps, channelBaroAirspeed, channelWind);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

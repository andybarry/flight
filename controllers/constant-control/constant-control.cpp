#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <iostream>
#include <string>


#include "../../LCM/lcmt_deltawing_u.h"
#include "lcmtypes/mav_pose_t.h" // from pronto

#include "../../externals/ConciseArgs.hpp"

using namespace std;

lcm_t * lcm;
lcmt_deltawing_u_subscription_t * mav_pose_sub;

const char *wingeron_u_channel;


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_deltawing_u_unsubscribe (lcm, mav_pose_sub);
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

void pose_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user)
{
    // publish control message

    lcmt_deltawing_u delta_u;

    delta_u.timestamp = getTimestampNow();
    delta_u.throttle = 0;
    delta_u.elevonL = 128;
    delta_u.elevonR = 128;
    delta_u.is_autonomous = 1;
    delta_u.video_record = 0;

    lcmt_deltawing_u_publish(lcm, wingeron_u_channel, &delta_u);
}


int main(int argc,char** argv)
{

    string pose_channel_str = "STATE_ESTIMATOR_POSE";
    string wingeron_u_channel_str = "wingeron_u";

    ConciseArgs parser(argc, argv);
    parser.add(pose_channel_str, "p", "pose-channel",
        "LCM channel for state estimate input");
    parser.add(wingeron_u_channel_str, "u", "wingeron-u-channel",
        "LCM channel for wingeron_u output");
    parser.parse();

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    wingeron_u_channel = wingeron_u_channel_str.c_str();

    mav_pose_sub =  lcmt_deltawing_u_subscribe (lcm, pose_channel_str.c_str(),
        &pose_handler, NULL);


    signal(SIGINT,sighandler);

    printf("Listening to LCM:\n\t%s\n", pose_channel_str.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

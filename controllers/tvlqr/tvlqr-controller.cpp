/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2014
 *
 */

#include "tvlqr-controller.hpp"

using namespace std;

lcm_t * lcm;

mav_pose_t_subscription_t *mav_pose_t_sub;

// global trajectory library
TrajectoryLibrary trajlib;

bot_lcmgl_t* lcmgl;

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {
    // todo
}


int main(int argc,char** argv) {

    bool ttl_one = false;
    string trajectory_dir = "";
    string pose_channel = "STATE_ESTIMATOR_POSE";

    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(trajectory_dir, "d", "trajectory-dir", "Directory containing CSV files with trajectories.", true);
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.parse();

    if (trajectory_dir != "") {
        // load a trajectory library
        if (!trajlib.LoadLibrary(trajectory_dir)) {
            cerr << "Error: failed to load trajectory library.  Quitting." << endl;
            return 1;
        }

        //trajlib.Print();
    }


    if (ttl_one) {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    } else {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    }

    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    mav_pose_t_sub = mav_pose_t_subscribe(lcm, pose_channel.c_str(), &mav_pose_t_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tState estimate: %s\n", pose_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

void sighandler(int dum)
{
    printf("\n\nclosing... ");

    mav_pose_t_unsubscribe(lcm, mav_pose_t_sub);
    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}

/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "tvlqr-controller.hpp"
#include "../../externals/ConciseArgs.hpp"

extern lcm_t * lcm;

extern mav_pose_t_subscription_t *mav_pose_t_sub;
extern lcmt_tvlqr_controller_action_subscription_t *tvlqr_controller_action_sub;

// global trajectory library
extern TrajectoryLibrary trajlib;

extern TvlqrControl *control;
extern ServoConverter *converter;

extern bot_lcmgl_t* lcmgl;

extern string deltawing_u_channel;

int main(int argc,char** argv) {

    bool ttl_one = false;
    string trajectory_dir = "";
    string pose_channel = "STATE_ESTIMATOR_POSE";
    string tvlqr_action_channel = "tvlqr-action";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(trajectory_dir, "d", "trajectory-dir", "Directory containing CSV files with trajectories.", true);
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.add(tvlqr_action_channel, "a", "tvlqr-channel", "LCM channel to listen for TVLQR action messages on.");
    parser.add(deltawing_u_channel, "u", "deltawing-u-channel", "LCM channel to send control messages on.");
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

    BotParam *param = bot_param_new_from_server(lcm, 0);

    ServoConverter *converter = new ServoConverter(param);

    control = new TvlqrControl(converter);


    mav_pose_t_sub = mav_pose_t_subscribe(lcm, pose_channel.c_str(), &mav_pose_t_handler, NULL);

    tvlqr_controller_action_sub = lcmt_tvlqr_controller_action_subscribe(lcm, tvlqr_action_channel.c_str(), &lcmt_tvlqr_controller_action_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    control->SetTrajectory(trajlib.GetTrajectoryByNumber(1));

    printf("Receiving LCM:\n\tState estimate: %s\n\tTVLQR action: %s\nSending LCM:\n\t%s\n", pose_channel.c_str(), tvlqr_action_channel.c_str(), deltawing_u_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

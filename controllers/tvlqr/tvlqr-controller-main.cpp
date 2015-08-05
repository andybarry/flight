/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "tvlqr-controller.hpp"
#include "../../externals/ConciseArgs.hpp"

extern lcm_t * lcm;

extern pronto_utime_t_subscription_t *pronto_reset_handler_sub;
extern mav_pose_t_subscription_t *mav_pose_t_sub;
extern lcmt_tvlqr_controller_action_subscription_t *tvlqr_controller_action_sub;
extern mav_filter_state_t_subscription_t *pronto_state_handler_sub;

// global trajectory library
extern TrajectoryLibrary trajlib;

extern TvlqrControl *control;
extern ServoConverter *converter;

extern bot_lcmgl_t* lcmgl;

extern string deltawing_u_channel;
extern string pronto_init_channel;
extern string pronto_reset_complete_channel;
extern string tvlqr_action_out_channel;

extern int number_of_switch_positions;
extern int switch_mapping[MAX_SWITCH_MAPPING];
extern int switch_rc_us[MAX_SWITCH_MAPPING];
extern int stable_controller;

extern double sigma0_vb;

extern double sigma0_delta_xy;
extern double sigma0_delta_z;

extern double sigma0_chi_xy;
extern double sigma0_chi_z;

int main(int argc,char** argv) {

    bool ttl_one = false;
    string trajectory_dir = "";
    string pose_channel = "STATE_ESTIMATOR_POSE";
    string tvlqr_action_channel = "tvlqr-action";
    string pronto_state_channel = "STATE_ESTIMATOR_STATE";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(trajectory_dir, "d", "trajectory-dir", "Directory containing CSV files with trajectories.", true);
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.add(tvlqr_action_channel, "a", "tvlqr-channel", "LCM channel to listen for TVLQR action messages on.");
    parser.add(tvlqr_action_out_channel, "o", "tvlqr-out-channel", "LCM channel to publish which TVLQR trajectory is running on.");
    parser.add(deltawing_u_channel, "u", "deltawing-u-channel", "LCM channel to send control messages on.");
    parser.add(pronto_init_channel, "i", "pronto-init-channel", "LCM channel to send pronto re-init messages on.");
    parser.add(pronto_state_channel, "s", "pronto-state-channel", "LCM channel that pronto publishes its state on.");
    parser.add(pronto_reset_complete_channel, "c", "pronto-reset-complete-channel", "LCM channel to listen for pronto's reset complete messages.");
    parser.parse();

    if (trajectory_dir != "") {
        // load a trajectory library
        if (!trajlib.LoadLibrary(trajectory_dir)) {
            std::cerr << "Error: failed to load trajectory library.  Quitting." << std::endl;
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

    if (param == NULL) {
        fprintf(stderr, "Error: no param server.  Quitting.\n");
        return 1;
    }

    ServoConverter *converter = new ServoConverter(param);

    // get sigma0 data
    sigma0_vb = bot_param_get_double_or_fail(param, "state_estimator.sigma0.vb");
    sigma0_delta_xy = bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_xy");
    sigma0_delta_z = bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_z");

    sigma0_chi_xy = bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_xy");
    sigma0_chi_xy = deg2rad(sigma0_chi_xy);

    sigma0_chi_z = bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_z");
    sigma0_chi_z = deg2rad(sigma0_chi_z);

    stable_controller = bot_param_get_int_or_fail(param, "tvlqr_controller.stable_controller");

    number_of_switch_positions = bot_param_get_int_or_fail(param, "tvlqr_controller.number_of_switch_positions");

    bot_param_get_int_array_or_fail(param, "tvlqr_controller.switch_rc_us", switch_rc_us, number_of_switch_positions);

    bot_param_get_int_array_or_fail(param, "tvlqr_controller.switch_mapping", switch_mapping, number_of_switch_positions);

    const Trajectory *stable_controller_traj = trajlib.GetTrajectoryByNumber(stable_controller);

    control = new TvlqrControl(converter, *stable_controller_traj);



    mav_pose_t_sub = mav_pose_t_subscribe(lcm, pose_channel.c_str(), &mav_pose_t_handler, NULL);

    pronto_reset_handler_sub = pronto_utime_t_subscribe(lcm, pronto_reset_complete_channel.c_str(), &pronto_reset_complete_handler, NULL);

    pronto_state_handler_sub = mav_filter_state_t_subscribe(lcm, pronto_state_channel.c_str(), &mav_filter_state_t_handler, NULL);

    tvlqr_controller_action_sub = lcmt_tvlqr_controller_action_subscribe(lcm, tvlqr_action_channel.c_str(), &lcmt_tvlqr_controller_action_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    control->SetTrajectory(*stable_controller_traj);

    printf("Receiving LCM:\n\tState estimate: %s\n\tTVLQR action: %s\nSending LCM:\n\t%s\n", pose_channel.c_str(), tvlqr_action_channel.c_str(), deltawing_u_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

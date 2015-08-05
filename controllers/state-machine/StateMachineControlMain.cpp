#include "StateMachineControl.hpp"
#include "../../externals/ConciseArgs.hpp"


int main(int argc,char** argv) {

    bool ttl_one = false;

    std::string trajectory_dir = "";
    std::string pose_channel = "STATE_ESTIMATOR_POSE";
    std::string stereo_channel = "stereo";

    std::string tvlqr_action_out_channel = "tvlqr-action-out";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(trajectory_dir, "d", "trajectory-dir", "Directory containing CSV files with trajectories.", true);
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.add(stereo_channel, "e", "stereo-channel", "LCM channel to listen to stereo messages on.");
    parser.add(tvlqr_action_out_channel, "o", "tvlqr-out-channel", "LCM channel to publish which TVLQR trajectory is running on.");

    parser.parse();


    std::string lcm_url;
    // create an lcm instance
    if (ttl_one) {
        lcm_url = "udpm://239.255.76.67:7667?ttl=1";
    } else {
        lcm_url = "udpm://239.255.76.67:7667?ttl=0";
    }
    lcm::LCM lcm(lcm_url);

    if (!lcm.good()) {
        std::cerr << "LCM creation failed." << std::endl;
        return 1;
    }

    // get parameter server
    BotParam *param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 0);
    BotFrames *bot_frames = bot_frames_get_global(lcm.getUnderlyingLCM(), param);

    double dist_threshold = bot_param_get_double_or_fail(param, "obstacle_avoidance.safe_distance_threshold");
    int start_traj_num = bot_param_get_int_or_fail(param, "tvlqr_controller.stable_controller");

    StateMachineControl fsm_control(&lcm, trajectory_dir, bot_frames, dist_threshold, start_traj_num, tvlqr_action_out_channel);

    // subscribe to LCM channels
    lcm.subscribe(pose_channel, &StateMachineControl::ProcessImuMsg, &fsm_control);
    lcm.subscribe(stereo_channel, &StateMachineControl::ProcessStereoMsg, &fsm_control);


    while (0 == lcm.handle()); // TODO: drop events if we're slow

    return 0;
}

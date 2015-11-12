#include "StateMachineControl.hpp"
#include "../../externals/ConciseArgs.hpp"


int main(int argc,char** argv) {

    bool ttl_one = false;
    bool visualization = false;
    bool traj_visualization = false;

    std::string pose_channel = "STATE_ESTIMATOR_POSE";
    std::string stereo_channel = "stereo";
    std::string rc_trajectory_commands_channel = "rc-trajectory-commands";
    std::string state_machine_go_autonomous_channel = "state-machine-go-autonomous";

    std::string tvlqr_action_out_channel = "tvlqr-action";
    std::string arm_for_takeoff_channel = "arm-for-takeoff";
    std::string state_message_channel = "state-machine-state";
    std::string altitude_reset_channel = "altitude-reset";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.add(stereo_channel, "e", "stereo-channel", "LCM channel to listen to stereo messages on.");
    parser.add(tvlqr_action_out_channel, "o", "tvlqr-out-channel", "LCM channel to publish which TVLQR trajectory is running on.");
    parser.add(rc_trajectory_commands_channel, "r", "rc-trajectory-commands-channel", "LCM channel to listen for RC trajectory commands on.");
    parser.add(state_machine_go_autonomous_channel, "a", "state-machine-go-autonomous-channel", "LCM channel to send go-autonmous messages on.");
    parser.add(visualization, "v", "visualization", "Enables visualization of obstacles for HUD / LCMGL.");
    parser.add(traj_visualization, "V", "traj-visualization", "Enables visualization of trajectories using LCMGL.");
    parser.add(arm_for_takeoff_channel, "A", "arm-for-takeoff-channel", "LCM channel to receive arm for takeoff messages on.");
    parser.add(state_message_channel, "s", "state-machine-state-channel", "LCM channel to send state machine state messages on.");

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

    BotParam *param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 0);

    std::string trajectory_dir = std::string(bot_param_get_str_or_fail(param, "tvlqr_controller.library_dir"));


    trajectory_dir = ReplaceUserVarInPath(trajectory_dir);

    StateMachineControl fsm_control(&lcm, trajectory_dir, tvlqr_action_out_channel, state_message_channel, altitude_reset_channel, visualization, traj_visualization);
    //fsm_control.GetFsmContext()->setDebugFlag(true);

    // subscribe to LCM channels
    lcm.subscribe(pose_channel, &StateMachineControl::ProcessImuMsg, &fsm_control);
    lcm.subscribe(stereo_channel, &StateMachineControl::ProcessStereoMsg, &fsm_control);
    lcm.subscribe(rc_trajectory_commands_channel, &StateMachineControl::ProcessRcTrajectoryMsg, &fsm_control);
    lcm.subscribe(state_machine_go_autonomous_channel, &StateMachineControl::ProcessGoAutonomousMsg, &fsm_control);
    lcm.subscribe(arm_for_takeoff_channel, &StateMachineControl::ProcessArmForTakeoffMsg, &fsm_control);

    omp_set_num_threads(3); // set the maximum number of threads
                            // to be 1 less than our number of
                            // cores so we never slow down
                            // the tvlqr process


    printf("Receiving LCM:\n\tPose: %s\n\tStereo: %s\n\tRC Trajectories: %s\n\tGo Autonomous: %s\n\tArm for Takeoff: %s\n\nSending LCM:\n\tTVLQR Action: %s\n\tState Machine State: %s\n\tAltitude reset: %s\n", pose_channel.c_str(), stereo_channel.c_str(), rc_trajectory_commands_channel.c_str(), state_machine_go_autonomous_channel.c_str(), arm_for_takeoff_channel.c_str(), tvlqr_action_out_channel.c_str(), state_message_channel.c_str(), altitude_reset_channel.c_str());

    while (true) {
        while (NonBlockingLcm(lcm.getUnderlyingLCM())) {}

        fsm_control.DoDelayedImuUpdate();
    }

    return 0;
}

#include "RcSwitchDispatch.hpp"
#include "../../externals/ConciseArgs.hpp"

RcSwitchDispatch::RcSwitchDispatch(std::string rc_trajectory_commands_channel, std::string stereo_channel, int stable_traj_num) {
    rc_trajectory_commands_channel_ = rc_trajectory_commands_channel;
    stereo_channel_ = stereo_channel;
}

void RcSwitchDispatch::ProcessRcTrajectoryMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::rc_switch_action *msg) {
    // new message has come in from the RC controller -- figure out what to do

    if (msg->action < 0) {
        // stabilization mode

    }
}


int main(int argc,char** argv) {

    bool ttl_one = false;

    std::string rc_action_channel = "rc-switch-action";

    std::string rc_trajectory_commands_channel = "rc-trajectory-commands";
    std::string stereo_channel = "stereo";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(stereo_channel, "s", "stereo-channel", "LCM channel to listen to stereo messages on.");
    parser.add(rc_trajectory_commands_channel, "t", "rc-trajectory-commands-channel", "LCM channel to sned RC trajectory commands on.");
    parser.add(rc_action_channel, "r", "rc-action-channel", "LCM channel to listen for RC switch actions on.");

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

    RcSwitchDispatch rc_dispatch(rc_trajectory_commands_channel, stereo_channel);

    // subscribe to LCM channels
    lcm.subscribe(rc_action_channel, &RcSwitchDispatch::ProcessRcTrajectoryMsg, &rc_dispatch);


    while (0 == lcm.handle());

    return 0;
}

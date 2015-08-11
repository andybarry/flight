/**
 * Performs dispatch for RC switch messages.
 *
 * (C) 2015 Andrew Barry <abarry@csail.mit.edu>
 */

#ifndef RC_SWITCH_DISPATH_HPP
#define RC_SWITCH_DISPATH_HPP


#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "../../LCM/lcmt/tvlqr_controller_action.hpp"
#include "../../LCM/lcmt/rc_switch_action.hpp"

#include <bot_param/param_client.h>

class RcSwitchDispatch {

    public:
        RcSwitchDispatch(std::string rc_trajectory_commands_channel, std::string stereo_channel, int stable_traj_num);

        void ProcessRcTrajectoryMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::rc_switch_action *msg);

    private:
        std::string stereo_channel_, rc_trajectory_commands_channel_;

        int stable_traj_num_;

};

#endif

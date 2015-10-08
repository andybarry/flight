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
#include "../../LCM/lcmt/stereo.hpp"
#include "../../LCM/lcmt/timestamp.hpp"
#include "../../LCM/lcmt/stereo_control.hpp"


#include "../../utils/utils/RealtimeUtils.hpp"
#include <bot_param/param_client.h>
#include <vector>

#define MAX_PARAM_SIZE 100

class RcSwitchDispatch {

    public:
        RcSwitchDispatch(lcm::LCM *lcm, std::string rc_trajectory_commands_channel, std::string stereo_channel, std::string state_machine_go_autonomous_channel, std::string stereo_control_channel, std::string arm_for_takeoff_channel);

        void ProcessRcMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::rc_switch_action *msg);

    private:
        void SendTrajectoryRequest(int traj_num) const;
        void SendStereoMsg(int stereo_msg_num) const;
        void SendGoAutonomousMsg() const;
        void SendArmForTakeoffMsg() const;
        void SendStereoWriteToDiskMsg() const;
        void DrakeToCameraFrame(double point_in[], double point_out[]) const;
        void SendStereoManyPoints(std::vector<float> x_in, std::vector<float> y_in, std::vector<float> z_in) const;
        int ServoMicroSecondsToSwitchPosition(int servo_value) const;

        std::string stereo_channel_, rc_trajectory_commands_channel_, state_machine_go_autonomous_channel_, stereo_control_channel_, arm_for_takeoff_channel_;

        int stable_traj_num_;
        int num_trajs_, num_stereo_actions_;
        int num_switch_positions_;
        int stop_stereo_pos_;
        int arm_for_takeoff_pos_;

        int switch_rc_us_[MAX_PARAM_SIZE];
        int trajectory_mapping_[MAX_PARAM_SIZE];
        int stereo_mapping_[MAX_PARAM_SIZE];

        lcm::LCM *lcm_;

        BotParam *param_;
        BotFrames *bot_frames_;

        bool stabilization_mode_;

        // pole to the left
        std::vector<float> x_points0_ = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
        std::vector<float> y_points0_ = { 2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2};
        std::vector<float> z_points0_ = { 4,  4,  4,  2,  2,  2,  0 , 0,  0, -2, -2, -2, -4, -4, -4};

        // pole to the right
        std::vector<float> x_points1_ = {10, 10, 10, 10, 10};
        std::vector<float> y_points1_ = {-2, -2, -2, -2, -2};
        std::vector<float> z_points1_ = {4,   2,  0, -2, -4};

        // pole below
        std::vector<float> x_points2_ = {10, 10, 10, 10, 10};
        std::vector<float> y_points2_ = {-4, -2,  0,  2,  4};
        std::vector<float> z_points2_ = {-3, -3, -3, -3, -3};


};

#endif

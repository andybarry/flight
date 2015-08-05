#ifndef STATE_MACHINE_CONTROL_HPP
#define STATE_MACHINE_CONTROL_HPP

/*
 * Implements a state machine controller for the aircraft
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "../../LCM/mav/pose_t.hpp"
#include "../../LCM/lcmt/stereo.hpp"
#include "../../LCM/lcmt/tvlqr_controller_action.hpp"
#include "AircraftStateMachine_sm.h"
#include <bot_param/param_client.h>
#include "../../controllers/TrajectoryLibrary/Trajectory.hpp"
#include "../../controllers/TrajectoryLibrary/TrajectoryLibrary.hpp"
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"

class StateMachineControl {

    public:
        StateMachineControl(lcm::LCM *lcm, std::string traj_dir, BotFrames *bot_frames, double dist_threshold, int start_traj_num, std::string tvlqr_action_out_channel);
        ~StateMachineControl();

        bool CheckForObstacles();

        void RequestTrajectory(const Trajectory &traj);

        void ProcessImuMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const mav::pose_t *msg);
        void ProcessStereoMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::stereo *msg);

        void SetLatestPose(const mav::pose_t &msg) { } // TODO


        mav::pose_t lastest_pose_;

    private:


        AircraftStateMachineContext fsm_;

        StereoOctomap *octomap_;
        TrajectoryLibrary *trajlib_;

        lcm::LCM *lcm_;


        BotFrames *bot_frames_;

        double safe_distance_;

        const Trajectory *current_traj_;
        int64_t traj_start_t_ = -1;

        std::string tvlqr_action_out_channel_;




};

#endif

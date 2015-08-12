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
#include "../../LCM/lcmt/timestamp.hpp"
#include "AircraftStateMachine_sm.h"
#include <bot_param/param_client.h>
#include "../../controllers/TrajectoryLibrary/Trajectory.hpp"
#include "../../controllers/TrajectoryLibrary/TrajectoryLibrary.hpp"
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"

class StateMachineControl {

    public:
        StateMachineControl(lcm::LCM *lcm, std::string traj_dir, std::string tvlqr_action_out_channel);
        ~StateMachineControl();

        bool IsObstacleInPath();

        const Trajectory GetCurrentTrajectory() { return *current_traj_; }
        const Trajectory GetStableTrajectory() { return *stable_traj_; }

        bool BetterTrajectoryAvailable();
        void RequestNewTrajectory();
        void SetBestTrajectory();

        void SetNextTrajectory(const Trajectory &traj) { next_traj_ = &traj; }
        void SetNextTrajectoryByNumber(int traj_num);

        void ProcessImuMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const mav::pose_t *msg);
        void ProcessStereoMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::stereo *msg);
        void ProcessRcTrajectoryMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::tvlqr_controller_action *msg);
        void ProcessGoAutonomousMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::timestamp *msg);


        bool CheckTrajectoryExpired();

        AircraftStateMachineContext* GetFsmContext() { return &fsm_; }
        const StereoOctomap* GetOctomap() const { return octomap_; }
        const TrajectoryLibrary* GetTrajectoryLibrary() const { return trajlib_; }

        std::string GetCurrentStateName() { return std::string(fsm_.getState().getName()); }

    private:


        AircraftStateMachineContext fsm_;

        StereoOctomap *octomap_;
        TrajectoryLibrary *trajlib_;

        lcm::LCM *lcm_;

        BotParam *param_;
        BotFrames *bot_frames_;

        double safe_distance_;
        double min_improvement_to_switch_trajs_;

        const Trajectory *current_traj_;

        const Trajectory *next_traj_;
        int64_t traj_start_t_ = -1;

        const Trajectory *stable_traj_;

        std::string tvlqr_action_out_channel_;




};

#endif

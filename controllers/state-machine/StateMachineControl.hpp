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
#include "../../LCM/lcmt/debug.hpp"
#include "AircraftStateMachine_sm.h"
#include <bot_param/param_client.h>
#include "../../controllers/TrajectoryLibrary/Trajectory.hpp"
#include "../../controllers/TrajectoryLibrary/TrajectoryLibrary.hpp"
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"
#include "../../estimators/SpacialStereoFilter/SpacialStereoFilter.hpp"

class StateMachineControl {

    public:
        StateMachineControl(lcm::LCM *lcm, std::string traj_dir, std::string tvlqr_action_out_channel, std::string state_message_channel, std::string altitude_reset_channel, bool visualization, bool traj_visualization);
        ~StateMachineControl();

        void DoDelayedImuUpdate();

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
        void ProcessArmForTakeoffMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::timestamp *msg);

        void SetTakeoffTime() { t_takeoff_ = ConvertTimestampToSeconds(GetTimestampNow()); }
        void SetTakeoffBearing() { desired_bearing_ = current_bearing_ + bearing_offset_; }

        bool CheckTrajectoryExpired();
        bool IsTakeoffAccel(const mav::pose_t &msg) const;
        bool HasClearedCable(const mav::pose_t &msg) const;
        bool ReachedCrusingAltitude(const mav::pose_t &msg) const;
        bool VelocityOkForThrottle(const mav::pose_t &msg) const;


        AircraftStateMachineContext* GetFsmContext() { return &fsm_; }
        const StereoOctomap* GetOctomap() const { return octomap_; }
        const TrajectoryLibrary* GetTrajectoryLibrary() const { return trajlib_; }

        std::string GetCurrentStateName() { return std::string(fsm_.getState().getName()); }

        int GetClimbNoThrottleTrajNum() const { return climb_no_throttle_trajnum_; }
        int GetClimbWithThrottleTrajNum() const { return climb_with_throttle_trajnum_; }

        void SendStateMsg(std::string state) const {
            lcmt::debug msg;
            msg.utime = GetTimestampNow();
            msg.debug = state;

            lcm_->publish(state_message_channel_, &msg);
        }

        void SendAltitudeResetMsg() const {
            lcmt::timestamp msg;
            msg.timestamp = GetTimestampNow();

            lcm_->publish(altitude_reset_channel_, &msg);
        }



    private:

        void PublishDebugMsg(std::string debug_str) const;
        int GetBearingPreferredTrajectoryNumber() const;

        AircraftStateMachineContext fsm_;

        StereoOctomap *octomap_;
        TrajectoryLibrary *trajlib_;

        SpacialStereoFilter *spacial_stereo_filter_;

        lcm::LCM *lcm_;

        BotParam *param_;
        BotFrames *bot_frames_;

        double safe_distance_;
        double min_improvement_to_switch_trajs_;
        double takeoff_threshold_x_;
        double takeoff_max_y_;
        double takeoff_max_z_;
        double ground_safety_distance_;

        double t_clear_cable_;
        double crusing_altitude_;
        double min_velocity_x_for_throttle_;

        double desired_bearing_;
        double bearing_tolerance_;
        double bearing_offset_;
        int traj_left_turn_, traj_right_turn_;

        double current_bearing_ = 0;
        bool bearing_init_ = false;

        double t_takeoff_ = -1;

        const Trajectory *current_traj_;

        const Trajectory *next_traj_;
        int64_t traj_start_t_ = -1;

        const Trajectory *stable_traj_;
        int climb_no_throttle_trajnum_;
        int climb_with_throttle_trajnum_;

        std::string tvlqr_action_out_channel_, state_message_channel_, altitude_reset_channel_;

        bool need_imu_update_;
        bool visualization_;
        bool traj_visualization_;

        mav::pose_t last_imu_msg_;

        BotTrans last_draw_transform_;

};

#endif

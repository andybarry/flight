#ifndef TVLQR_CONTROL_HPP
#define TVLQR_CONTROL_HPP

/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>

#include "../TrajectoryLibrary/Trajectory.hpp"
#include "../../LCM/mav_pose_t.h"

#include <Eigen/Core>

#include "../../utils/utils/RealtimeUtils.hpp"

#include "../../utils/ServoConverter/ServoConverter.hpp"


class TvlqrControl
{

    public:
        TvlqrControl(const ServoConverter *converter, Trajectory *stable_controller);

        void SetTrajectory(Trajectory *trajectory);

        bool HasTrajectory() { return current_trajectory_ != NULL; }

        Eigen::VectorXi GetControl(const mav_pose_t *msg);

        void SetStateEstimatorInitialized();

        bool IsTimeInvariant() { return HasTrajectory() && current_trajectory_->IsTimeInvariant(); }

    private:

        void InitializeState(const mav_pose_t *msg);
        double GetTNow();
        Eigen::VectorXd GetStateMinusInit(const mav_pose_t *msg);

        Trajectory *current_trajectory_;
        Trajectory *stable_controller_;

        Eigen::VectorXd initial_state_;
        Eigen::VectorXd last_state_; // keep so we can do angle unwrapping
        Eigen::Matrix3d Mz_; // rotation matrix that transforms global state into local state by removing yaw

        bool state_initialized_;

        const ServoConverter *converter_;

        int64_t t0_;
        double last_ti_state_estimator_reset_;


};

#endif

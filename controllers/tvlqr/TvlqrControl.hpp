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

#include <Eigen/Core>

#include "../../utils/utils/RealtimeUtils.hpp"

#include "../../utils/ServoConverter/ServoConverter.hpp"

using namespace std;


class TvlqrControl
{

    public:
        TvlqrControl(ServoConverter *converter);

        void SetTrajectory(Trajectory *trajectory);

        bool HasTrajectory() { return current_trajectory_ != NULL; }

        Eigen::VectorXi GetControl(Eigen::VectorXd state);

        void StateEstimatorResetFinished();


    private:

        void InitializeState(Eigen::VectorXd state);
        double GetTNow();
        Eigen::VectorXd GetStateMinusInit(Eigen::VectorXd state);

        Trajectory *current_trajectory_;
        Eigen::VectorXd initial_state_;
        bool state_initialized_;

        ServoConverter *converter_;

        int64_t t0_;

};

#endif

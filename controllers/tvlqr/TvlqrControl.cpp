/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "TvlqrControl.hpp"

TvlqrControl::TvlqrControl(const ServoConverter *converter, Trajectory *stable_controller) {
    current_trajectory_ = NULL;
    state_initialized_ = false;
    t0_ = 0;
    last_ti_state_estimator_reset_ = 0;
    converter_ = converter;
    stable_controller_ = stable_controller;
}


void TvlqrControl::SetTrajectory(Trajectory *trajectory) {

    if (trajectory == NULL) {
        cerr << "Warning: NULL trajectory in SetTrajectory." << endl;
    }

    current_trajectory_ = trajectory;

    state_initialized_ = false;

}

Eigen::VectorXi TvlqrControl::GetControl(const mav_pose_t *msg) {

    if (current_trajectory_ == NULL) {
        cerr << "Warning: NULL trajectory in GetControl." << endl;
        return converter_->GetTrimCommands();
    }

    //std::cout << "in GetControl" << std::endl;

    // check to see if this is the first state we've gotten along this trajectory

    if (state_initialized_ == false) {

        InitializeState(msg);
    }

    Eigen::VectorXd state_minus_init = GetStateMinusInit(msg);
/*
    printf("BEFORE\troll: %f\tpitch: %f\tyaw: %f\n", state_minus_init(3), state_minus_init(4), state_minus_init(5));

    // unwrap angles
    state_minus_init(3) = AngleUnwrap(state_minus_init(3), last_state_(3));
    state_minus_init(4) = AngleUnwrap(state_minus_init(4), last_state_(4));
    state_minus_init(5) = AngleUnwrap(state_minus_init(5), last_state_(5));

    printf("AFTER\troll: %f\tpitch: %f\tyaw: %f\n", state_minus_init(3), state_minus_init(4), state_minus_init(5));

    last_state_ = state_minus_init;
*/
    double t_along_trajectory;

    // check for TILQR case
    if (current_trajectory_->IsTimeInvariant()) {
        t_along_trajectory = 0;
    } else {
        t_along_trajectory = GetTNow();
    }

    if (t_along_trajectory <= current_trajectory_->GetMaxTime()) {

        Eigen::VectorXd x0 = current_trajectory_->GetState(t_along_trajectory);
        Eigen::MatrixXd gain_matrix = current_trajectory_->GetGainMatrix(t_along_trajectory);

        Eigen::VectorXd state_error = state_minus_init - x0;

        //cout << "state error = " << endl << state_error << endl;

        Eigen::VectorXd additional_control_action = gain_matrix * state_error;

        //cout << "additional control action = " << endl << additional_control_action << endl;

//cout << "t = " << t_along_trajectory << endl;
//cout << "gain" << endl << gain_matrix << endl << "state_error" << endl << state_error << endl << "additional" << endl << additional_control_action << endl;

        Eigen::VectorXd command_in_rad = current_trajectory_->GetUCommand(t_along_trajectory) + additional_control_action;

//cout << "command_in_rad" << endl << command_in_rad << endl;

        return converter_->RadiansToServoCommands(command_in_rad);
    } else {
        // we are past the max time, return stabilizing controller
        SetTrajectory(stable_controller_);
        return GetControl(msg);

        //return converter_->GetTrimCommands();

    }
}

void TvlqrControl::InitializeState(const mav_pose_t *msg) {

    initial_state_ = PoseMsgToStateEstimatorVector(msg);

    // get the yaw from the initial state

    double rpy[3];

    bot_quat_to_roll_pitch_yaw(msg->orientation, rpy);

    Mz_ = rotz(-rpy[2]);

    t0_ = GetTimestampNow();

    state_initialized_ = true;

}

Eigen::VectorXd TvlqrControl::GetStateMinusInit(const mav_pose_t *msg) {

    // subtract out x0, y0, z0

    mav_pose_t *msg2 = mav_pose_t_copy(msg);

    msg2->pos[0] -= initial_state_(0); // x
    msg2->pos[1] -= initial_state_(1); // y
    msg2->pos[2] -= initial_state_(2); // z

    Eigen::VectorXd state = PoseMsgToStateEstimatorVector(msg2, Mz_);

    mav_pose_t_destroy(msg2);

    last_state_ = state;

    return state;

}

double TvlqrControl::GetTNow() {

    int64_t delta_t = GetTimestampNow() - t0_;

    // convert to seconds
    return double(delta_t) / 1000000.0;

}


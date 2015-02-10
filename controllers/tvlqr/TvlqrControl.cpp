/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "TvlqrControl.hpp"

TvlqrControl::TvlqrControl(ServoConverter *converter) {
    current_trajectory_ = NULL;
    state_initialized_ = false;
    t0_ = 0;
    converter_ = converter;
}


void TvlqrControl::SetTrajectory(Trajectory *trajectory) {

    if (trajectory == NULL) {
        cerr << "Warning: NULL trajectory in SetTrajectory." << endl;
    }

    current_trajectory_ = trajectory;

    state_initialized_ = false;

}

Eigen::VectorXi TvlqrControl::GetControl(Eigen::VectorXd state) {

    if (current_trajectory_ == NULL) {
        cerr << "Warning: NULL trajectory in GetControl." << endl;
        return converter_->GetTrimCommands();
    }

    // check to see if this is the first state we've gotten along this trajectory

    if (state_initialized_ == false) {

        InitializeState(state);
    }

    Eigen::VectorXd state_minus_init = GetStateMinusInit(state);

    double t_along_trajectory = GetTNow();

    if (t_along_trajectory < current_trajectory_->GetMaxTime()) {

        Eigen::VectorXd x0 = current_trajectory_->GetState(t_along_trajectory);
        Eigen::MatrixXd gain_matrix = current_trajectory_->GetGainMatrix(t_along_trajectory);

        Eigen::VectorXd state_error = state_minus_init - x0;

gain_matrix(0,0) = 0;
gain_matrix(1,0) = 0;
gain_matrix(2,0) = 0;

gain_matrix(0,1) = 0;
gain_matrix(1,1) = 0;
gain_matrix(2,1) = 0;

gain_matrix(0,2) = 0;
gain_matrix(1,2) = 0;
gain_matrix(2,2) = 0;

gain_matrix(0,6) = 0;
gain_matrix(1,6) = 0;
gain_matrix(2,6) = 0;

        Eigen::VectorXd additional_control_action = gain_matrix * state_error;

//cout << "t = " << t_along_trajectory << endl;
//cout << "gain" << endl << gain_matrix << endl << "state_error" << endl << state_error << endl << "additional" << endl << additional_control_action << endl;

        Eigen::VectorXd command_in_rad = current_trajectory_->GetUCommand(t_along_trajectory) + additional_control_action;

//cout << "command_in_rad" << endl << command_in_rad << endl;

        return converter_->RadiansToServoCommands(command_in_rad);
    } else {
        // we are past the max time, return nominal trims

        return converter_->GetTrimCommands();

    }
}

void TvlqrControl::InitializeState(Eigen::VectorXd state) {

    initial_state_ = state;

    t0_ = GetTimestampNow();

    state_initialized_ = true;

}

Eigen::VectorXd TvlqrControl::GetStateMinusInit(Eigen::VectorXd state) {

    // subtract out x0, y0, z0 and yaw0

    Eigen::VectorXd state_minus_init = state;

    state_minus_init(0) -= initial_state_(0); // x
    state_minus_init(1) -= initial_state_(1); // y
    state_minus_init(2) -= initial_state_(2); // z

    state_minus_init(5) -= initial_state_(5); //yaw

    return state_minus_init;

}

double TvlqrControl::GetTNow() {

    int64_t delta_t = GetTimestampNow() - t0_;

    // convert to seconds
    return double(delta_t) / 1000000.0;

}


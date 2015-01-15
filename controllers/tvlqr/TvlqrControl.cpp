/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "TvlqrControl.hpp"

TvlqrControl::TvlqrControl() {
    current_trajectory_ = NULL;
}


void TvlqrControl::SetTrajectory(Trajectory *trajectory, Eigen::VectorXd initial_state) {

    if (trajectory == NULL) {
        cerr << "Warning: NULL trajectory in SetTrajectory." << endl;
    }

    current_trajectory_ = trajectory;

    // capture the initial state so we know how to deal with it down the line

    initial_state_ = initial_state;

    // get a transform from the initial state to the global origin

    // TODO?
}

Eigen::VectorXd TvlqrControl::GetControl(double t_along_trajectory, Eigen::VectorXd state) {

    if (current_trajectory_ == NULL) {
        cerr << "Warning: NULL trajectory in GetControl." << endl;
    }

    Eigen::VectorXd x0 = current_trajectory_->GetState(t_along_trajectory);


    Eigen::MatrixXd gain_matrix = current_trajectory_->GetGainMatrix(t_along_trajectory);

    Eigen::VectorXd additional_control_action = gain_matrix * (state - x0);

    return current_trajectory_->GetUCommand(t_along_trajectory) + additional_control_action;

}

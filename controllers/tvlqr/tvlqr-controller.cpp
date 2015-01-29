/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "tvlqr-controller.hpp"

using namespace std;

lcm_t * lcm;

mav_pose_t_subscription_t *mav_pose_t_sub;
lcmt_tvlqr_controller_action_subscription_t *tvlqr_controller_action_sub;

// global trajectory library
TrajectoryLibrary trajlib;

TvlqrControl control;

bot_lcmgl_t* lcmgl;

int64_t traj_timestamp;

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {

    // whenever we get a state estimate, we want to output a new control action

    cout << "handler" << endl;

    Eigen::VectorXd state_vec = StateEstimatorToDrakeVector(msg);


}

Eigen::VectorXd StateEstimatorToDrakeVector(const mav_pose_t *msg) {

    // convert message to 12-state vector in the Drake frame

    Eigen::VectorXd state(12);

    // x, y, and z are direct
    state(0) = msg->pos[0];
    state(1) = msg->pos[1];
    state(2) = msg->pos[2];

    // roll, pitch, and yaw come from the quats
    double rpy[3];

    bot_quat_to_roll_pitch_yaw(msg->orientation, rpy);

    state(3) = rpy[0];
    state(4) = rpy[1];
    state(5) = rpy[2];

    Eigen::Vector3d UVW; // in body frame
    UVW(0) = msg->vel[0];
    UVW(1) = msg->vel[1];
    UVW(2) = msg->vel[2];

    Eigen::Vector3d rpy_eigen(rpy);

    // velocities are given in the body frame, we need to move them
    // to the global frame

    Eigen::Matrix3d R_body_to_world = rpy2rotmat(rpy_eigen);
    //Eigen::Matrix3d R_world_to_body = R_body_to_world.Transpose();

    Eigen::Vector3d vel_world_frame = R_body_to_world * UVW;

    state(6) = vel_world_frame(0);
    state(7) = vel_world_frame(1);
    state(8) = vel_world_frame(2);

    // rotation rates are given in body frame

    Eigen::Vector3d PQR;
    PQR(0) = msg->rotation_rate[0];
    PQR(1) = msg->rotation_rate[1];
    PQR(2) = msg->rotation_rate[2];

    Eigen::Vector3d pqr = R_body_to_world * PQR;

    // convert rotation rates into rolldot, pitchdot, yawdot

    Eigen::Vector3d rpydot = angularvel2rpydot(rpy_eigen, pqr);

    state(9) = rpydot(0);
    state(10) = rpydot(1);
    state(11) = rpydot(2);

    return state;

}




void lcmt_tvlqr_controller_action_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_tvlqr_controller_action *msg, void *user) {

    // getting an action means we should start a new TVLQR controller!

    Trajectory *traj = trajlib.GetTrajectoryByNumber(msg->trajectory_number);

    if (traj == NULL) {
        cerr << "Warning: trajectory number " << msg->trajectory_number << " was NULL!  Aborting trajectory run." << endl;
        return;
    }

    control.SetTrajectory(traj);

    traj_timestamp = GetTimestampNow();


    cout << "Starting trajectory " << msg->trajectory_number << " at t = " << traj_timestamp << endl;


}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    mav_pose_t_unsubscribe(lcm, mav_pose_t_sub);
    lcmt_tvlqr_controller_action_unsubscribe(lcm, tvlqr_controller_action_sub);

    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}

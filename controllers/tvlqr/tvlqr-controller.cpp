/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "tvlqr-controller.hpp"

lcm_t * lcm;

mav_pose_t_subscription_t *mav_pose_t_sub;
lcmt_tvlqr_controller_action_subscription_t *tvlqr_controller_action_sub;
pronto_utime_t_subscription_t *pronto_reset_handler_sub;
mav_filter_state_t_subscription_t *pronto_state_handler_sub;

// global trajectory library
TrajectoryLibrary trajlib;

TvlqrControl *control;
ServoConverter *converter;

bot_lcmgl_t* lcmgl;
string deltawing_u_channel = "deltawing_u";
string pronto_init_channel = "MAV_STATE_EST_INITIALIZER";
string pronto_reset_complete_channel = "MAV_STATE_EST_INIT_COMPLETE";
string tvlqr_action_out_channel = "tvlqr-action-out";

int stable_controller;

bool state_estimator_init = true;

mav_pose_t *last_pose_msg;

mav_filter_state_t *last_filter_state = NULL;

double sigma0_vb;

double sigma0_delta_xy;
double sigma0_delta_z;

double sigma0_chi_xy;
double sigma0_chi_z;

int64_t last_ti_state_estimator_reset = 0;


void pronto_reset_complete_handler(const lcm_recv_buf_t *rbuf, const char* channel, const pronto_utime_t *msg, void *user) {

    // a pronto-reset has happened!  Charge forward with the new trajectory
    state_estimator_init = true;

    std::cout << "Got state estimator reset" << std::endl;


}

void mav_filter_state_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_filter_state_t *msg, void *user) {

    if (last_filter_state != NULL) {
        mav_filter_state_t_destroy(last_filter_state);
    }

    last_filter_state = mav_filter_state_t_copy(msg);

}

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {

    if (!control->HasTrajectory()) {

        return;
    }

    if (!state_estimator_init) {
        std::cout << "Waiting for state estimator init..." << std::endl;
        return;
    }

    if (last_pose_msg != NULL) {
        mav_pose_t_destroy(last_pose_msg);
    }
    last_pose_msg = mav_pose_t_copy(msg);

    // whenever we get a state estimate, we want to output a new control action

    // check for time-invariant trajectory requiring state estimator reset
    if (control->IsTimeInvariant() && GetTimestampNow() - last_ti_state_estimator_reset > 500000) {

        last_ti_state_estimator_reset = GetTimestampNow();

        SendStateEstimatorResetRequest();
    }

    Eigen::VectorXi control_vec = control->GetControl(msg);

    // send control out through LCM

    lcmt_deltawing_u u_msg;

    u_msg.timestamp = GetTimestampNow();

    u_msg.elevonL = control_vec(0);
    u_msg.elevonR = control_vec(1);
    u_msg.throttle = control_vec(2);

    u_msg.is_autonomous = true;
    u_msg.video_record = true; // todo, this isn't quite how this should be designed

    lcmt_deltawing_u_publish(lcm, deltawing_u_channel.c_str(), &u_msg);

}

void lcmt_tvlqr_controller_action_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_tvlqr_controller_action *msg, void *user) {
    int lib_num = msg->trajectory_number;

    // getting an action means we should start a new TVLQR controller!
    const Trajectory *traj = trajlib.GetTrajectoryByNumber(lib_num);

    if (traj == NULL) {
        std::cerr << "Warning: trajectory number " << lib_num << " was NULL!  Aborting trajectory run." << std::endl;
        return;
    }

    lcmt_tvlqr_controller_action msg_out;
    msg_out.timestamp = GetTimestampNow();
    msg_out.trajectory_number = lib_num;
    lcmt_tvlqr_controller_action_publish(lcm, tvlqr_action_out_channel.c_str(), &msg_out);

    control->SetTrajectory(*traj);


    std::cout << "Starting trajectory " << lib_num << std::endl;

    state_estimator_init = false;

    // request a state estimator init

    SendStateEstimatorResetRequest();


}

void SendStateEstimatorResetRequest() {
    if (last_filter_state == NULL) {
        SendStateEstimatorDefaultResetRequest();
        return;
    }

    // if we are here, then we have an old state to copy from

    mav_filter_state_t *reset_request = mav_filter_state_t_copy(last_filter_state);

    // reset the covariances on position (x, y)

    // x-position row
    for (int i = 189; i <= 209; i++) {
        reset_request->cov[i] = 0;
    }

    // x-position column
    for (int i = 9; i <= 429; i+=21) {
        reset_request->cov[i] = 0;
    }

    // y-position row
    for (int i = 210; i <= 230; i++) {
        reset_request->cov[i] = 0;
    }

    // y-position column
    for (int i = 10; i <= 430; i+=21) {
        reset_request->cov[i] = 0;
    }

    reset_request->cov[198] = sigma0_delta_xy * sigma0_delta_xy;
    reset_request->cov[220] = sigma0_delta_xy * sigma0_delta_xy;


    mav_filter_state_t_publish(lcm, pronto_init_channel.c_str(), reset_request);

    std::cout << "State estimator reset message sent." << std::endl;

    mav_filter_state_t_destroy(reset_request);
}

void SendStateEstimatorDefaultResetRequest() {

    mav_filter_state_t msg;

    msg.utime = GetTimestampNow();

    // copy in the states from the last position, but reset the covariance
    if (last_pose_msg != NULL) {
        msg.quat[0] = last_pose_msg->orientation[0];
        msg.quat[1] = last_pose_msg->orientation[1];
        msg.quat[2] = last_pose_msg->orientation[2];
        msg.quat[3] = last_pose_msg->orientation[3];
    } else {
        msg.quat[0] = 1;
        msg.quat[1] = 0;
        msg.quat[2] = 0;
        msg.quat[3] = 0;
    }


    msg.num_states = 21;

    double states[msg.num_states];

    if (last_pose_msg != NULL) {
        states[0] = last_pose_msg->rotation_rate[0];
        states[1] = last_pose_msg->rotation_rate[1];
        states[2] = last_pose_msg->rotation_rate[2];

        states[3] = last_pose_msg->vel[0];
        states[4] = last_pose_msg->vel[1];
        states[5] = last_pose_msg->vel[2];

    } else {
        states[0] = 0;
        states[1] = 0;
        states[2] = 0;

        states[3] = 0;
        states[4] = 0;
        states[5] = 0;
    }

    states[6] = 0;
    states[7] = 0;
    states[8] = 0;

    if (last_pose_msg != NULL) {

        states[9] = last_pose_msg->pos[0];
        states[10] = last_pose_msg->pos[1];
        states[11] = last_pose_msg->pos[2];

        states[12] = last_pose_msg->accel[0];
        states[13] = last_pose_msg->accel[1];
        states[14] = last_pose_msg->accel[2];

    } else {
        states[9] = 0;
        states[10] = 0;
        states[11] = 0;

        states[12] = 0;
        states[13] = 0;
        states[14] = 0;
    }



    states[15] = 0;
    states[16] = 0;
    states[17] = 0;
    states[18] = 0;
    states[19] = 0;
    states[20] = 0;

    msg.state = states;

    msg.num_cov_elements = 441;

    double covs[msg.num_cov_elements];

    for (int i = 0; i < msg.num_cov_elements; i++) {
        covs[i] = 0;
    }

    covs[66] = sigma0_vb * sigma0_vb;
    covs[88] = sigma0_vb * sigma0_vb;
    covs[110] = sigma0_vb * sigma0_vb;
    covs[132] = sigma0_chi_xy * sigma0_chi_xy;
    covs[154] = sigma0_chi_xy * sigma0_chi_xy;
    covs[176] = sigma0_chi_z * sigma0_chi_z;
    covs[198] = sigma0_delta_xy * sigma0_delta_xy;
    covs[220] = sigma0_delta_xy * sigma0_delta_xy;
    covs[242] = sigma0_delta_z * sigma0_delta_z;

    msg.cov = covs;

    mav_filter_state_t_publish(lcm, pronto_init_channel.c_str(), &msg);

    std::cout << "State estimator DEFAULT reset message sent." << std::endl;
}

void sighandler(int dum)
{
    printf("\n\nclosing... ");

    mav_pose_t_unsubscribe(lcm, mav_pose_t_sub);
    lcmt_tvlqr_controller_action_unsubscribe(lcm, tvlqr_controller_action_sub);
    mav_filter_state_t_unsubscribe(lcm, pronto_state_handler_sub);

    lcm_destroy (lcm);

    if (converter != NULL) {
        delete converter;
    }

    if (control != NULL) {
        delete control;
    }

    printf("done.\n");

    exit(0);
}

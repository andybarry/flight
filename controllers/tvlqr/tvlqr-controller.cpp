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
pronto_utime_t_subscription_t *pronto_reset_handler_sub;

// global trajectory library
TrajectoryLibrary trajlib;

TvlqrControl *control;
ServoConverter *converter;

bot_lcmgl_t* lcmgl;
string deltawing_u_channel = "deltawing_u";
string pronto_init_channel = "MAV_STATE_EST_INITIALIZER";
string pronto_reset_complete_channel = "MAV_STATE_EST_INIT_COMPLETE";
bool state_estimator_init = true;

mav_pose_t last_pose_msg;

void pronto_reset_complete_handler(const lcm_recv_buf_t *rbuf, const char* channel, const pronto_utime_t *msg, void *user) {

    // a pronto-reset has happened!  Charge forward with the new trajectory
    state_estimator_init = true;


}

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {

    if (!control->HasTrajectory()) {

        return;
    }

    if (!state_estimator_init) {
        cout << "Waiting for state estimator init..." << endl;
        return;
    }

    SavePoseMsg(msg);

    // whenever we get a state estimate, we want to output a new control action

    Eigen::VectorXd state_vec = StateEstimatorToDrakeVector(msg);


    Eigen::VectorXi control_vec = control->GetControl(state_vec);

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

    // getting an action means we should start a new TVLQR controller!

    Trajectory *traj = trajlib.GetTrajectoryByNumber(msg->trajectory_number);

    if (traj == NULL) {
        cerr << "Warning: trajectory number " << msg->trajectory_number << " was NULL!  Aborting trajectory run." << endl;
        return;
    }

    control->SetTrajectory(traj);


    cout << "Starting trajectory " << msg->trajectory_number << endl;

    state_estimator_init = false;

    // request a state estimator init

    SendStateEstimatorResetRequest();


}

void SavePoseMsg(const mav_pose_t *msg) {

    last_pose_msg.utime = msg->utime;

    last_pose_msg.pos[0] = msg->pos[0];
    last_pose_msg.pos[1] = msg->pos[1];
    last_pose_msg.pos[2] = msg->pos[2];

    last_pose_msg.vel[0] = msg->vel[0];
    last_pose_msg.vel[1] = msg->vel[1];
    last_pose_msg.vel[2] = msg->vel[2];

    last_pose_msg.orientation[0] = msg->orientation[0];
    last_pose_msg.orientation[1] = msg->orientation[1];
    last_pose_msg.orientation[2] = msg->orientation[2];
    last_pose_msg.orientation[3] = msg->orientation[3];

    last_pose_msg.rotation_rate[0] = msg->rotation_rate[0];
    last_pose_msg.rotation_rate[1] = msg->rotation_rate[1];
    last_pose_msg.rotation_rate[2] = msg->rotation_rate[2];

    last_pose_msg.accel[0] = msg->accel[0];
    last_pose_msg.accel[1] = msg->accel[1];
    last_pose_msg.accel[2] = msg->accel[2];

}

void SendStateEstimatorResetRequest() {

    mav_filter_state_t msg;

    msg.utime = GetTimestampNow();

    // copy in the states from the last position, but reset the covariance
    msg.quat[0] = last_pose_msg.orientation[0];
    msg.quat[1] = last_pose_msg.orientation[1];
    msg.quat[2] = last_pose_msg.orientation[2];
    msg.quat[3] = last_pose_msg.orientation[3];


    msg.num_states = 21;

    double states[msg.num_states];

    states[0] = last_pose_msg.rotation_rate[0];
    states[1] = last_pose_msg.rotation_rate[1];
    states[2] = last_pose_msg.rotation_rate[2];

    states[3] = last_pose_msg.vel[0];
    states[4] = last_pose_msg.vel[1];
    states[5] = last_pose_msg.vel[2];

    states[6] = 0;
    states[7] = 0;
    states[8] = 0;

    states[9] = last_pose_msg.pos[0];
    states[10] = last_pose_msg.pos[1];
    states[11] = last_pose_msg.pos[2];

    states[12] = last_pose_msg.accel[0];
    states[13] = last_pose_msg.accel[1];
    states[14] = last_pose_msg.accel[2];

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

    covs[66] = 0.0225;
    covs[88] = 0.0225;
    covs[110] = 0.025;
    covs[132] = 0.00274155677;
    covs[154] = 0.00274155677;
    covs[176] = 0.00274155677;
    covs[198] = 0.25;
    covs[220] = 0.25;

    msg.cov = covs;

    mav_filter_state_t_publish(lcm, pronto_init_channel.c_str(), &msg);

    cout << "State estimator reset message sent." << endl;
}

void sighandler(int dum)
{
    printf("\n\nclosing... ");

    mav_pose_t_unsubscribe(lcm, mav_pose_t_sub);
    lcmt_tvlqr_controller_action_unsubscribe(lcm, tvlqr_controller_action_sub);

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

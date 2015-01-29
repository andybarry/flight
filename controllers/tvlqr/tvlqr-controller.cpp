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

string deltawing_u_channel = "wingeron_u";

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {

    // whenever we get a state estimate, we want to output a new control action

    cout << "handler" << endl;

    Eigen::VectorXd state_vec = StateEstimatorToDrakeVector(msg);


    Eigen::VectorXd control_vec = control.GetControl(state_vec);

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

    control.SetTrajectory(traj);


    cout << "Starting trajectory " << msg->trajectory_number << endl;


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

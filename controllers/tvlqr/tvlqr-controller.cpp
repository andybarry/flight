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

Eigen::Matrix3d rpy2rotmat(Eigen::Vector3d rpy) {

    // from Drake's rpy2rotmat

    Eigen::Matrix3d rotmat;

    rotmat(0,0) = cos(rpy(2))*cos(rpy(1));
    rotmat(0,1) = cos(rpy(2))*sin(rpy(1))*sin(rpy(0))-sin(rpy(2))*cos(rpy(0));
    rotmat(0,2) = cos(rpy(2))*sin(rpy(1))*cos(rpy(0))+sin(rpy(2))*sin(rpy(0));

    rotmat(1,0) = sin(rpy(2))*cos(rpy(1));
    rotmat(1,1) = sin(rpy(2))*sin(rpy(1))*sin(rpy(0))+cos(rpy(2))*cos(rpy(0));
    rotmat(1,2) = sin(rpy(2))*sin(rpy(1))*cos(rpy(0))-cos(rpy(2))*sin(rpy(0));

    rotmat(2,0) = -sin(rpy(1));
    rotmat(2,1) = cos(rpy(1))*sin(rpy(0));
    rotmat(2,2) = cos(rpy(1))*cos(rpy(0));

    return rotmat;


}

Eigen::Vector3d angularvel2rpydot(Eigen::Vector3d rpy, Eigen::Vector3d omega) {

    // from Drake

    return angularvel2rpydotMatrix(rpy) * omega;

}

Eigen::Matrix3d angularvel2rpydotMatrix(Eigen::Vector3d rpy) {

    // from Drake

    double p = rpy(1);
    double y = rpy(2);

    double sy = sin(y);
    double cy = cos(y);
    double sp = sin(p);
    double cp = cos(p);
    double tp = sp / cp;

    Eigen::Matrix3d phi;

    phi(0,0) = cy/cp;
    phi(0,1) = sy/cp;
    phi(0,2) = 0;

    phi(1,0) = -sy;
    phi(1,1) = cy;
    phi(1,2) = 0;

    phi(2,0) = cy*tp;
    phi(2,1) = tp*sy;
    phi(2,2) = 1;

    return phi;

}


int64_t GetTimestampNow() {
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
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


int main(int argc,char** argv) {

    bool ttl_one = false;
    string trajectory_dir = "";
    string pose_channel = "STATE_ESTIMATOR_POSE";
    string tvlqr_action_channel = "tvlqr-action";

    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(trajectory_dir, "d", "trajectory-dir", "Directory containing CSV files with trajectories.", true);
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.add(tvlqr_action_channel, "a", "tvlqr-channel", "LCM channel to listen for TVLQR action messages on.");
    parser.parse();

    if (trajectory_dir != "") {
        // load a trajectory library
        if (!trajlib.LoadLibrary(trajectory_dir)) {
            cerr << "Error: failed to load trajectory library.  Quitting." << endl;
            return 1;
        }

        //trajlib.Print();
    }


    if (ttl_one) {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    } else {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    }

    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    mav_pose_t_sub = mav_pose_t_subscribe(lcm, pose_channel.c_str(), &mav_pose_t_handler, NULL);

    tvlqr_controller_action_sub = lcmt_tvlqr_controller_action_subscribe(lcm, tvlqr_action_channel.c_str(), &lcmt_tvlqr_controller_action_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    control.SetTrajectory(trajlib.GetTrajectoryByNumber(1));

    printf("Receiving LCM:\n\tState estimate: %s\n\tTVLQR action: %s\n", pose_channel.c_str(), tvlqr_action_channel.c_str());

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
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

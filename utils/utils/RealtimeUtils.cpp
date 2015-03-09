#include "RealtimeUtils.hpp"

#define PI 3.14159265359

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

TEST(Utils, StateEstimatorToDrakeVector) {

    mav_pose_t msg;

    msg.pos[0] = 0.5079;
    msg.pos[1] = 0.0855;
    msg.pos[2] = 0.2625;

    msg.orientation[0] = 0.8258;
    msg.orientation[1] = 0.3425;
    msg.orientation[2] = 0.1866;
    msg.orientation[3] = 0.4074;

    msg.vel[0] = 0.7303;
    msg.vel[1] = 0.4886;
    msg.vel[2] = 0.5785;

    msg.rotation_rate[0] = 0.2373;
    msg.rotation_rate[1] = 0.4588;
    msg.rotation_rate[2] = 0.9631;


    Eigen::VectorXd output = StateEstimatorToDrakeVector(&msg);

    Eigen::VectorXd matlab_output(12);

    matlab_output << 0.5079, 0.0855, 0.2625, 0.8010, 0.0292, 0.9289, 0.5106, 0.5572, 0.7318, 0.2665, -0.3722, 1.0002;


    EXPECT_TRUE( output.isApprox(matlab_output, 0.001) ) << std::endl << "Expected:" << std::endl << matlab_output << std::endl << "Got:" << std::endl << output << std::endl;


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

TEST(Utils, rpy2rotmat) {

    Eigen::Vector3d test_vec;
    test_vec(0) = 0;
    test_vec(1) = 0;
    test_vec(2) = 0;

    EXPECT_TRUE(rpy2rotmat(test_vec) == Eigen::Matrix3d::Identity()) << "Zeros for rpy should result in eye(3).";

    test_vec(0) = 0.1111;
    test_vec(1) = 0.2581;
    test_vec(2) = 0.4087;

    Eigen::Matrix3d result = rpy2rotmat(test_vec);

    Eigen::Matrix3d result_from_matlab;
    result_from_matlab << 0.8872,   -0.3690,    0.2768,
            0.3843,    0.9232,   -0.0008,
            -0.2552,    0.1071,    0.9609;


    EXPECT_TRUE(result.isApprox(result_from_matlab, 0.001)) << "Expected:" << std::endl <<result << std::endl << "Actual:" << std::endl << result_from_matlab;

}

Eigen::Vector3d angularvel2rpydot(Eigen::Vector3d rpy, Eigen::Vector3d omega) {

    // from Drake

    return angularvel2rpydotMatrix(rpy) * omega;

}

TEST(Utils, AngularVel2Rpy) {

    Eigen::Vector3d tester;
    tester << 0.2967, .3188, .4242;

    Eigen::Matrix3d matlab_output;
    matlab_output << 0.9597,    0.4334,         0,
                    -0.4116,    0.9114,         0,
                    0.3008,    0.1358,    1.0000;

    Eigen::Matrix3d result = angularvel2rpydotMatrix(tester);

    EXPECT_TRUE(result.isApprox(matlab_output, 0.0001));

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

TEST(Utils, AngularVel2RpyDot) {

    Eigen::Vector3d tester1, tester2;
    tester1 << 0.5949, 0.2622, 0.6028;

    tester2 << 0.7112, 0.2217, 0.1174;


    Eigen::Vector3d output = angularvel2rpydot(tester1, tester2);

    Eigen::Vector3d matlab_output;
    matlab_output << 0.7367, -0.2206, 0.3084;

    EXPECT_TRUE(matlab_output.isApprox(output, 0.0001));

}


int64_t GetTimestampNow() {
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

TEST(Utils, TimestampSanity2015) {

    EXPECT_TRUE(GetTimestampNow() > 1422487159500367) << "Timestamp should be after Jan 28, 2015.";
}

double deg2rad(double input_in_deg) {
    return PI/180.0d * input_in_deg;
}

TEST(Utils, deg2rad) {

    EXPECT_NEAR( 0.0374446428, deg2rad(2.14542), 0.0001 );

    EXPECT_EQ(0, deg2rad(0));

    EXPECT_NEAR(-1.14664641, deg2rad(-65.698), 0.0001);
}


/**
 * Processes LCM messages without blocking.
 *
 * @param lcm lcm object
 *
 * @retval true if processed a message
 */
bool NonBlockingLcm(lcm_t *lcm)
{
    // setup an lcm function that won't block when we read it
    int lcm_fd = lcm_get_fileno(lcm);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);

    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        0,  // seconds
        1   // microseconds
    };


    int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

    if(0 == status) {
        // no messages
        //do nothing
        return false;

    } else if(FD_ISSET(lcm_fd, &fds)) {
        // LCM has events ready to be processed.
        lcm_handle(lcm);
        return true;
    }
    return false;

}


#include "RealtimeUtils.hpp"

#define PI 3.14159265359


Eigen::VectorXd PoseMsgToStateEstimatorVector(const mav_pose_t *msg, const Eigen::Matrix3d Mz) {
    // convert message to 12-state vector in the State estimator frame

    Eigen::VectorXd state(12);

    Eigen::Vector3d pos_eigen;


    pos_eigen(0) = msg->pos[0];
    pos_eigen(1) = msg->pos[1];
    pos_eigen(2) = msg->pos[2];

    // x, y, and z are always in global frame (add in any custom rotation)
    Eigen::Vector3d pos_global = Mz * pos_eigen;

    state(0) = pos_global(0);
    state(1) = pos_global(1);
    state(2) = pos_global(2);

    // roll, pitch, and yaw come from the quats in the message

    Eigen::Vector4d q_eigen;
    q_eigen(0) = msg->orientation[0];
    q_eigen(1) = msg->orientation[1];
    q_eigen(2) = msg->orientation[2];
    q_eigen(3) = msg->orientation[3];

    double rpy_array[3];
    bot_quat_to_roll_pitch_yaw(msg->orientation, rpy_array);

    Eigen::Matrix3d rot_mat = quat2rotmat(q_eigen);

    Eigen::Vector3d rpy = rotmat2rpy(Mz * rot_mat);

    state(3) = rpy(0);
    state(4) = rpy(1);
    state(5) = rpy(2);

    // velocities are in body frame
    state(6) = msg->vel[0];
    state(7) = msg->vel[1];
    state(8) = msg->vel[2];

    // rotation rates are given in body frame
    // as angular velocities

    state(9) = msg->rotation_rate[0];
    state(10) = msg->rotation_rate[1];
    state(11) = msg->rotation_rate[2];

    return state;
}


TEST(Utils, PoseMsgToStateEstimatorVector) {

    mav_pose_t msg;

    msg.pos[0] = 0.1829;
    msg.pos[1] = 0.2399;
    msg.pos[2] = 0.8865;

    msg.orientation[0] = 0.9669;
    msg.orientation[1] = -0.0065;
    msg.orientation[2] = 0.2428;
    msg.orientation[3] = 0.0779;

    msg.vel[0] = 0.9787;
    msg.vel[1] = 0.7127;
    msg.vel[2] = 0.5005;

    msg.rotation_rate[0] = 0.4711;
    msg.rotation_rate[1] = 0.0596;
    msg.rotation_rate[2] = 0.6820;


    Eigen::VectorXd output = PoseMsgToStateEstimatorVector(&msg);

    Eigen::VectorXd matlab_output(12);

    matlab_output << 0.1829, 0.2399, 0.8865, 0.0287, 0.4899, 0.1679, 0.9787, 0.7127, 0.5005, 0.4711, 0.0596, 0.6820;


    EXPECT_TRUE( output.isApprox(matlab_output, 0.001) ) << std::endl << "Expected:" << std::endl << matlab_output << std::endl << "Got:" << std::endl << output << std::endl;

    //Eigen::VectorXd matlab_output2 = matlab_output;
    //matlab_output2(4) = matlab_output(4) + 2*PI;

    //output = PoseMsgToStateEstimatorVector(&msg, &matlab_output2);
    //EXPECT_TRUE( output.isApprox(matlab_output2, 0.001) ) << std::endl << "(testing angle unwrap) Expected:" << std::endl << matlab_output << std::endl << "Got:" << std::endl << output << std::endl;

    //matlab_output2(4) = matlab_output(4) + .3*PI;

    //output = PoseMsgToStateEstimatorVector(&msg, &matlab_output2);
    //EXPECT_TRUE( output.isApprox(matlab_output, 0.001) ) << std::endl << "(testing angle unwrap2) Expected:" << std::endl << matlab_output << std::endl << "Got:" << std::endl << output << std::endl;

}

TEST(Utils, PoseMsgToStateEstimatorVectorMz) {

    mav_pose_t msg;

    msg.pos[0] = 0.1829;
    msg.pos[1] = 0.2399;
    msg.pos[2] = 0.8865;

    msg.orientation[0] = 0.9669;
    msg.orientation[1] = -0.0065;
    msg.orientation[2] = 0.2428;
    msg.orientation[3] = 0.0779;

    msg.vel[0] = 0.9787;
    msg.vel[1] = 0.7127;
    msg.vel[2] = 0.5005;

    msg.rotation_rate[0] = 0.4711;
    msg.rotation_rate[1] = 0.0596;
    msg.rotation_rate[2] = 0.6820;

    double rpy[3];
    bot_quat_to_roll_pitch_yaw(msg.orientation, rpy);
    Eigen::Matrix3d rotz_mat = rotz(-rpy[2]);


    Eigen::VectorXd output = PoseMsgToStateEstimatorVector(&msg, rotz_mat);

    Eigen::VectorXd matlab_output(12);

    matlab_output << 0.2205, 0.2060, 0.8865, 0.0287, 0.4899, 0, 0.9787, 0.7127, 0.5005, 0.4711, 0.0596, 0.6820;


    EXPECT_TRUE( output.isApprox(matlab_output, 0.001) ) << std::endl << "Expected:" << std::endl << matlab_output << std::endl << "Got:" << std::endl << output << std::endl;

}

double AngleUnwrap(double angle_rad_in, double last_angle_rad) {

    // compute 5 options:
    //  1: no change
    //  2: -360 deg
    //  3: +360 deg
    //  4: -360*2 deg
    //  5: +360*2 ded

    double angle_options[10];
    angle_options[0] = angle_rad_in;
    angle_options[1] = angle_rad_in - 2*PI;
    angle_options[2] = angle_rad_in + 2*PI;
    angle_options[3] = angle_rad_in - 4*PI;
    angle_options[4] = angle_rad_in + 4*PI;

    double best_angle = angle_rad_in;
    double best_dist = -1;

    // find the minimum distance
    for (int i = 0; i < 5; i++) {

        double this_dist = abs(last_angle_rad - angle_options[i]);

        if (best_dist < 0 || this_dist < best_dist) {
            best_dist = this_dist;

            best_angle = angle_options[i];
        }


    }

    return best_angle;
}

TEST(Utils, AngleUnwrap) {



    EXPECT_NEAR( 1.0, AngleUnwrap(1.0, 0.9), 0.0001);
    EXPECT_NEAR( 2*PI + .01, AngleUnwrap(0.01, 2*PI), 0.0001 );
    EXPECT_NEAR( -2*PI + .01, AngleUnwrap(0.01, -2*PI), 0.0001 );
    EXPECT_NEAR( 2*PI, AngleUnwrap(2*PI, 6.0), 0.0001 );
    EXPECT_NEAR( -2*PI-.1, AngleUnwrap(-.1, -2*PI), 0.0001 );

    EXPECT_NEAR( .78, AngleUnwrap(.78, .77), 0.0001);

    EXPECT_NEAR( 3.2732, AngleUnwrap(-3.01, 3.11), 0.0001);
}



Eigen::VectorXd StateEstimatorToDrakeVector(const mav_pose_t *msg, const Eigen::Matrix3d Mz) {

    // convert message to 12-state vector in the Drake frame

    Eigen::VectorXd state(12);

    Eigen::Vector3d pos_eigen;


    pos_eigen(0) = msg->pos[0];
    pos_eigen(1) = msg->pos[1];
    pos_eigen(2) = msg->pos[2];

    // x, y, and z are direct (already in global frame), excepting any custom rotation
    Eigen::Vector3d pos_global = Mz * pos_eigen;

    state(0) = pos_global(0);
    state(1) = pos_global(1);
    state(2) = pos_global(2);

    // roll, pitch, and yaw come from the quats in the message

    Eigen::Vector4d q_eigen;
    q_eigen(0) = msg->orientation[0];
    q_eigen(1) = msg->orientation[1];
    q_eigen(2) = msg->orientation[2];
    q_eigen(3) = msg->orientation[3];

    double rpy_array[3];
    bot_quat_to_roll_pitch_yaw(msg->orientation, rpy_array);

    Eigen::Vector3d rpy_eigen(rpy_array);

    Eigen::Matrix3d rot_mat = quat2rotmat(q_eigen);

    Eigen::Vector3d rpy = rotmat2rpy(Mz * rot_mat);

    state(3) = rpy(0);
    state(4) = rpy(1);
    state(5) = rpy(2);

    Eigen::Vector3d UVW; // in body frame
    UVW(0) = msg->vel[0];
    UVW(1) = msg->vel[1];
    UVW(2) = msg->vel[2];



    // velocities are given in the body frame, we need to move them
    // to the global frame

    Eigen::Matrix3d R_body_to_world = rpy2rotmat(rpy_eigen);
    //Eigen::Matrix3d R_world_to_body = R_body_to_world.Transpose();

    Eigen::Vector3d vel_world_frame = Mz * R_body_to_world * UVW;

    state(6) = vel_world_frame(0);
    state(7) = vel_world_frame(1);
    state(8) = vel_world_frame(2);

    // rotation rates are given in body frame

    Eigen::Vector3d PQR;
    PQR(0) = msg->rotation_rate[0];
    PQR(1) = msg->rotation_rate[1];
    PQR(2) = msg->rotation_rate[2];

    Eigen::Vector3d pqr = Mz * R_body_to_world * PQR;

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

TEST(Utils, StateEstimatorToDrakeVectorMz) {

    mav_pose_t msg;

    msg.pos[0] = 643;
    msg.pos[1] =  -19.59;
    msg.pos[2] = -14.06;

    msg.orientation[0] = 0.92042916327673;
    msg.orientation[1] = 0.075662452847708837;
    msg.orientation[2] = -0.3131459922391228;
    msg.orientation[3] = -0.22142871003294068;

    msg.vel[0] = 12.358;
    msg.vel[1] = -0.37176;
    msg.vel[2] = -7.321641;

    msg.rotation_rate[0] = -0.00431;
    msg.rotation_rate[1] = 0.02377;
    msg.rotation_rate[2] = 1.199e-4;


    double rpy[3];

    bot_quat_to_roll_pitch_yaw(msg.orientation, rpy);

    Eigen::Matrix3d rotz_mat = rotz(-rpy[2]);

    Eigen::VectorXd output = StateEstimatorToDrakeVector(&msg, rotz_mat);

    Eigen::VectorXd matlab_output(12);

    matlab_output << 551.0521, 331.9253, -14.0600, 0.3374, -0.5739,  0, 14.1958, 2.0726, 0.8045, -0.0095, 0.0224, 0.0095;


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

Eigen::Matrix3d quat2rotmat(Eigen::Vector4d q) {

    double qdotq = q.dot(q);

    double norm_q = sqrt(qdotq);

    Eigen::Vector4d q_norm = q/norm_q;


    double w = q_norm(0);
    double x = q_norm(1);
    double y = q_norm(2);
    double z = q_norm(3);

    Eigen::Matrix3d rot_mat;

    rot_mat << w*w + x*x - y*y - z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y,
        2*x*y + 2*w*z,  w*w + y*y - x*x - z*z, 2*y*z - 2*w*x,
        2*x*z - 2*w*y, 2*y*z + 2*w*x, w*w + z*z - x*x - y*y;

    return rot_mat;

}

TEST(Utils, quat2rotmat) {

    Eigen::Vector4d q;

    q << -0.9355137319401352, 0.14485829821440904, -0.23100540100248626, -0.2246478037392905;

    Eigen::Matrix3d output = quat2rotmat(q);

    double q_array[4];
    q_array[0] = q(0);
    q_array[1] = q(1);
    q_array[2] = q(2);
    q_array[3] = q(3);

    double rpy[3];

    bot_quat_to_roll_pitch_yaw(q_array, rpy);

    Eigen::Vector3d rpy_eigen;
    rpy_eigen(0) = rpy[0];
    rpy_eigen(1) = rpy[1];
    rpy_eigen(2) = rpy[2];

    Eigen::Matrix3d other_functions_output = rpy2rotmat(rpy_eigen);


    EXPECT_TRUE(other_functions_output.isApprox(output, 0.0001));


    Eigen::Vector4d q2;
    q2 << 0.8177, 0.3668, 0.2438, 0.3708;

    Eigen::Matrix3d output2 = quat2rotmat(q2);

    Eigen::Matrix3d matlab_output;
    matlab_output << 0.6062, -0.4275, 0.6707,
        0.7852, 0.4560, -0.4190,
        -0.1267, 0.7806, 0.6121;


    EXPECT_TRUE(matlab_output.isApprox(output2, 0.0001));


}

Eigen::Vector3d rotmat2rpy(Eigen::Matrix3d R) {

    Eigen::Vector3d rpy;

    rpy(0) = atan2(R(2,1), R(2,2));
    rpy(1) = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
    rpy(2) = atan2(R(1,0),R(0,0));

    return rpy;
}

TEST(Utils, rotmat2rpy) {

    Eigen::Matrix3d rot_mat;
    rot_mat <<  0.9569,   -0.1664,    0.2380,
                0.2736,    0.7914,   -0.5467,
               -0.0974,    0.5882,    0.8028;

    Eigen::Vector3d output = rotmat2rpy(rot_mat);

    Eigen::Vector3d matlab_output;
    matlab_output << 0.6323, 0.0976, 0.2785;

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

    EXPECT_NEAR(0, deg2rad(0), 0.00001) << "Issue with deg2rad(0)";

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

Eigen::Matrix3d rotz(double rotation_around_z) {

    Eigen::Matrix3d rot_mat;

    double s = sin(rotation_around_z);
    double c = cos(rotation_around_z);

    rot_mat(0,0) = c;
    rot_mat(0,1) = -s;
    rot_mat(0,2) = 0;

    rot_mat(1,0) = s;
    rot_mat(1,1) = c;
    rot_mat(1,2) = 0;

    rot_mat(2,0) = 0;
    rot_mat(2,1) = 0;
    rot_mat(2,2) = 1;

    return rot_mat;
}


TEST(Utils, rotz) {

    double theta = 0.8147;

    Eigen::Matrix3d matlab_output;
    matlab_output << 0.6861, -0.7275, 0, 0.7275, 0.6861, 0, 0, 0, 1;


    Eigen::Matrix3d output = rotz(theta);

    EXPECT_TRUE(matlab_output.isApprox(output, 0.0001));


    double theta2 = 3.235;
    Eigen::Matrix3d matlab_output2;
    matlab_output2 << -0.9956, 0.0933, 0, -0.0933, -0.9956, 0, 0, 0, 1;

    Eigen::Matrix3d output2 = rotz(theta2);

    EXPECT_TRUE(matlab_output2.isApprox(output2, 0.0001));

}

void DrawOriginLcmGl(lcm_t *lcm) {

    bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm, "Origin");
    bot_lcmgl_line_width(lcmgl, 2.0f);

    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, 0, 0, 0);
    bot_lcmgl_vertex3d(lcmgl, 1, 0, 0);
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_color3f(lcmgl, 0, 1, 0);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, 0, 0, 0);
    bot_lcmgl_vertex3d(lcmgl, 0, 1, 0);
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_color3f(lcmgl, 0, 0, 1);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, 0, 0, 0);
    bot_lcmgl_vertex3d(lcmgl, 0, 0, 1);
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_switch_buffer(lcmgl);
}

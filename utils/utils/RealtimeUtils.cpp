#include "RealtimeUtils.hpp"

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


    EXPECT_TRUE(result.isApprox(result_from_matlab, 0.001)) << "Expected:" << endl <<result << endl << "Actual:" << endl << result_from_matlab;

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



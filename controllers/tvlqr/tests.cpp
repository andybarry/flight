#include "tvlqr-controller.hpp"
#include "gtest/gtest.h"

TEST(tvlqr, StateEstimatorToDrakeVector) {

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


    EXPECT_TRUE( output.isApprox(matlab_output, 0.001) ) << endl << "Expected:" << endl << matlab_output << endl << "Got:" << endl << output << endl;


}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

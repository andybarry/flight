#include "ServoConverter.hpp"
#include "gtest/gtest.h"
#include <lcm/lcm.h>
#include <lcm/lcm_coretypes.h>

class ServoConverterTest : public testing::Test {

    protected:

        virtual void SetUp() {
            // start up lcm

            lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

            param_ = bot_param_new_from_server(lcm_, 0);

            converter_ = new ServoConverter(param_);

        }

        virtual void TearDown() {
            lcm_destroy(lcm_);
            // todo: delete param_;
            delete converter_;
        }

        lcm_t *lcm_;
        BotParam *param_;

        ServoConverter *converter_;

};

TEST_F(ServoConverterTest, RadiansToServo) {

    Eigen::Vector3d tester;

    tester << 0.5235, -2.1, 3.99;

    Eigen::Vector3i matlab_output;

    matlab_output << 1672, 2517, 1610;

    Eigen::Vector3i output = converter_->RadiansToServoCommands(tester);

    EXPECT_TRUE(matlab_output == output) << "Expected:" << endl << matlab_output << endl << "Actual:" << endl << output;

}

TEST_F(ServoConverterTest, RadiansToServo2) {

    Eigen::Vector3d tester;

    tester << 0.2, 2, 4.8058;

    Eigen::Vector3i matlab_output;

    matlab_output << 1518, 1565, 1690;

    Eigen::Vector3i output = converter_->RadiansToServoCommands(tester);

    EXPECT_TRUE(matlab_output == output) << "Expected:" << endl << matlab_output << endl << "Actual:" << endl << output;

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

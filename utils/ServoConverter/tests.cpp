#include "ServoConverter.hpp"
#include "gtest/gtest.h"
#include <lcm/lcm.h>
#include <lcm/lcm_coretypes.h>

class ServoConverterTest : public testing::Test {

    protected:

        virtual void SetUp() {
            // start up lcm

            lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

            param_ = bot_param_new_from_file("../../config/plane-odroid-gps1.cfg");

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

    tester << 0.5235, -1.1, 3.99;

    Eigen::Vector3i matlab_output;

    matlab_output << 1672, 2082, 1610;

    Eigen::Vector3i output = converter_->RadiansToServoCommands(tester);

    EXPECT_TRUE(matlab_output == output) << "Expected:" << std::endl << matlab_output << std::endl << "Actual:" << std::endl << output;

}

TEST_F(ServoConverterTest, RadiansToServo2) {

    Eigen::Vector3d tester;

    tester << 0.2, 0.2, 4.8058;

    Eigen::Vector3i matlab_output;

    matlab_output << 1519, 1517, 1691;

    Eigen::Vector3i output = converter_->RadiansToServoCommands(tester);

    EXPECT_TRUE(matlab_output == output) << "Expected:" << std::endl << matlab_output << std::endl << "Actual:" << std::endl << output;

}

TEST_F(ServoConverterTest, MinMaxCommands) {

    Eigen::Vector3i tester;

    tester << 1518, -570, 2800;

    Eigen::Vector3i matlab_output;

    matlab_output << 1518, 1050, 1892;

    Eigen::Vector3i output = converter_->MinMaxCommands(tester);

    EXPECT_TRUE(matlab_output == output) << "Expected:" << std::endl << matlab_output << std::endl << "Actual:" << std::endl << output;

}

TEST_F(ServoConverterTest, MinMaxCommands2) {

    Eigen::Vector3i tester;

    tester << 1518, 152421, -124;

    Eigen::Vector3i matlab_output;

    matlab_output << 1518, 2090, 1103;

    Eigen::Vector3i output = converter_->MinMaxCommands(tester);

    EXPECT_TRUE(matlab_output == output) << "Expected:" << std::endl << matlab_output << std::endl << "Actual:" << std::endl << output;

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

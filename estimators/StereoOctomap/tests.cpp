#include "StereoOctomap.hpp"
#include "gtest/gtest.h"
#include "../../LCM/lcmt_stereo.h"
#include "../../utils/utils/RealtimeUtils.hpp"

class StereoOctomapTest : public testing::Test {

    protected:

        virtual void SetUp() {
            // start up lcm

            lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

            param_ = bot_param_new_from_server(lcm_, 0);
            BotFrames *bot_frames = bot_frames_new(lcm_, param_);

            stereo_octomap_ = new StereoOctomap(bot_frames);

        }

        virtual void TearDown() {
            lcm_destroy(lcm_);
            // todo: delete param_;
            delete stereo_octomap_;
        }

        lcm_t *lcm_;
        BotParam *param_;

        StereoOctomap *stereo_octomap_;

};

TEST_F(StereoOctomapTest, SimpleNearestNeighbor) {

    // first test when no points are there

    double origin[3];
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;

    EXPECT_TRUE(stereo_octomap_->NearestNeighbor(origin) == -1) << "No points in octomap failed." << std::endl;

    // add a point

    lcmt_stereo msg;

    msg.timestamp = GetTimestampNow();

    float x[1];
    float y[1];
    float z[1];

    x[0] = 1;
    y[0] = 0;
    z[0] = 0;

    msg.x = x;
    msg.y = y;
    msg.z = z;

    msg.number_of_points = 1;
    msg.frame_number = 0;
    msg.video_number = 0;

    stereo_octomap_->ProcessStereoMessage(&msg);


    double point[3];
    point[0] = 1;
    point[1] = 0;
    point[2] = 0;

    EXPECT_EQ(stereo_octomap_->NearestNeighbor(origin), 1);

    EXPECT_EQ(stereo_octomap_->NearestNeighbor(point), 0);




}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

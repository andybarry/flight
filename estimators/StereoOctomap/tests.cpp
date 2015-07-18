#include "StereoOctomap.hpp"
#include "gtest/gtest.h"
#include "../../LCM/lcmt_stereo.h"
#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/mav_pose_t.h"

class StereoOctomapTest : public testing::Test {

    public:
        void SendAtOriginMessage() {
            mav_pose_t msg;
            msg.utime = GetTimestampNow();

            msg.pos[0] = 0;
            msg.pos[1] = 0;
            msg.pos[2] = 0;

            msg.vel[0] = 0;
            msg.vel[1] = 0;
            msg.vel[2] = 0;

            msg.orientation[0] = 1;
            msg.orientation[1] = 0;
            msg.orientation[2] = 0;
            msg.orientation[3] = 0;

            msg.rotation_rate[0] = 0;
            msg.rotation_rate[1] = 0;
            msg.rotation_rate[2] = 0;

            msg.accel[0] = 0;
            msg.accel[1] = 0;
            msg.accel[2] = 0;

            mav_pose_t_publish(lcm_, "STATE_ESTIMATOR_POSE", &msg);
        }

    protected:

        virtual void SetUp() {
            // start up lcm

            lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

            param_ = bot_param_new_from_server(lcm_, 0);
            bot_frames_ = bot_frames_get_global(lcm_, param_);

            stereo_octomap_ = new StereoOctomap(bot_frames_);

        }

        virtual void TearDown() {
            lcm_destroy(lcm_);
            // todo: delete param_;
            delete stereo_octomap_;
        }

        lcm_t *lcm_;
        BotParam *param_;;
        BotFrames *bot_frames_;

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
    double point[3], trans_point[3];
    point[0] = 1;
    point[1] = 0;
    point[2] = 0;


    // figure out what this point (which is currently expressed in global coordinates
    // will be in local opencv coordinates
    BotTrans open_cv_trans;
    bot_frames_get_trans(bot_frames_, "local", "opencvFrame", &open_cv_trans);

    bot_trans_apply_vec(&open_cv_trans, point, trans_point);

    lcmt_stereo msg;

    msg.timestamp = GetTimestampNow();

    float x[1];
    float y[1];
    float z[1];

    x[0] = trans_point[0];
    y[0] = trans_point[1];
    z[0] = trans_point[2];

    msg.x = x;
    msg.y = y;
    msg.z = z;

    msg.number_of_points = 1;
    msg.frame_number = 0;
    msg.video_number = 0;

    stereo_octomap_->ProcessStereoMessage(&msg);


    EXPECT_EQ(stereo_octomap_->NearestNeighbor(origin), 1);

    EXPECT_EQ(stereo_octomap_->NearestNeighbor(point), 0);

    double point2[3] = {0, 0, 1};

    EXPECT_NEAR(stereo_octomap_->NearestNeighbor(point2), sqrt(2), 0.00001);




}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

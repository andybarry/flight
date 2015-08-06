#include "StereoOctomap.hpp"
#include "gtest/gtest.h"
#include "../../LCM/lcmt_stereo.h"
#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/mav_pose_t.h"
#include <ctime>
#include <stack>

#define TOLERANCE 0.0001


class StereoOctomapTest : public testing::Test {

    protected:

        virtual void SetUp() {
            // start up lcm

            lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

            param_ = bot_param_new_from_server(lcm_, 0);
            bot_frames_ = bot_frames_new(lcm_, param_);


            bot_frames_get_trans(bot_frames_, "local", "opencvFrame", &global_to_camera_trans_);
            bot_frames_get_trans(bot_frames_, "opencvFrame", "local", &camera_to_global_trans_);

        }

        virtual void TearDown() {
            lcm_destroy(lcm_);
            // todo: delete param_;
        }


        lcm_t *lcm_;
        BotParam *param_;
        BotFrames *bot_frames_;
        BotTrans global_to_camera_trans_, camera_to_global_trans_;

        double NearestNeighborLinear(vector<float> x, vector<float> y, vector<float> z, double query_point[]) {

            double best_dist = -1;
            int num_points = x.size();


            for (int i = 0; i < num_points; i++) {
                double this_point_camera_frame[3];

                this_point_camera_frame[0] = x[i];
                this_point_camera_frame[1] = y[i];
                this_point_camera_frame[2] = z[i];

                double this_point[3];

                CameraToGlobalFrame(this_point_camera_frame, this_point);


                double this_dist = sqrt(pow(this_point[0] - query_point[0], 2) + pow(this_point[1] - query_point[1], 2) + pow(this_point[2] - query_point[2], 2));

                if (this_dist < best_dist || best_dist < 0) {
                    best_dist = this_dist;
                }
            }

            return best_dist;
        }

        void GlobalToCameraFrame(double point_in[], double point_out[]) {
            // figure out what this point (which is currently expressed in global coordinates
            // will be in local opencv coordinates

            bot_trans_apply_vec(&global_to_camera_trans_, point_in, point_out);
        }

        void CameraToGlobalFrame(double point_in[], double point_out[]) {
            bot_trans_apply_vec(&camera_to_global_trans_, point_in, point_out);
        }

        std::stack<double> tictoc_wall_stack;

        void tic() {
            tictoc_wall_stack.push(GetTimestampNow() / 1000000.0);
        }

        double toc() {
            double outval = GetTimestampNow() / 1000000.0 - tictoc_wall_stack.top();
            tictoc_wall_stack.pop();
            return outval;
        }



};

TEST_F(StereoOctomapTest, SimpleNearestNeighbor) {

    StereoOctomap *stereo_octomap = new StereoOctomap(bot_frames_);

    // first test when no points are there

    double origin[3];
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;

    EXPECT_TRUE(stereo_octomap->NearestNeighbor(origin) == -1) << "No points in octomap failed." << std::endl;

    // add a point
    double point[3], trans_point[3];
    point[0] = 1;
    point[1] = 0;
    point[2] = 0;


    GlobalToCameraFrame(point, trans_point);

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    vector<float> x, y, z;

    x.push_back(trans_point[0]);
    y.push_back(trans_point[1]);
    z.push_back(trans_point[2]);

    msg.x = x;
    msg.y = y;
    msg.z = z;

    msg.number_of_points = 1;
    msg.frame_number = 0;
    msg.video_number = 0;

    stereo_octomap->ProcessStereoMessage(&msg);


    EXPECT_EQ_ARM(stereo_octomap->NearestNeighbor(origin), 1);

    EXPECT_EQ_ARM(stereo_octomap->NearestNeighbor(point), 0);

    double point2[3] = {0, 0, 1};

    EXPECT_NEAR(stereo_octomap->NearestNeighbor(point2), sqrt(2), TOLERANCE);


    lcmt::stereo msg2;
    msg2.timestamp = GetTimestampNow();

    vector<float> x_2, y_2, z_2;

    double trans_point2[3];

    point[0] = 0;
    point[1] = 2;
    point[2] = 0;
    GlobalToCameraFrame(point, trans_point2);

    x_2.push_back(trans_point2[0]);
    y_2.push_back(trans_point2[1]);
    z_2.push_back(trans_point2[2]);

    msg2.x = x_2;
    msg2.y = y_2;
    msg2.z = z_2;

    msg2.number_of_points = 1;
    msg2.frame_number = 0;
    msg2.video_number = 0;

    // add the points to the octomap
    stereo_octomap->ProcessStereoMessage(&msg2);

    // check

    EXPECT_NEAR(stereo_octomap->NearestNeighbor(origin), 1, TOLERANCE);

    double query[3] = { 1.5, 0, 0 };

    EXPECT_NEAR(stereo_octomap->NearestNeighbor(query), 0.5, TOLERANCE);

    query[0] = 1;
    query[1] = 0;
    query[2] = 1;

    // check z
    EXPECT_NEAR(stereo_octomap->NearestNeighbor(query), 1, TOLERANCE);

    query[0] = 0;
    query[1] = 2.01;
    query[2] = 0;

    EXPECT_NEAR(stereo_octomap->NearestNeighbor(query), 0.01, TOLERANCE);

    query[0] = 0;
    query[1] = 2.1;
    query[2] = 0.5;

    EXPECT_NEAR(stereo_octomap->NearestNeighbor(query), sqrt(0.1*0.1 + 0.5*0.5), TOLERANCE);

    query[0] = -1;
    query[1] = 2.5;
    query[2] = 1.5;

    EXPECT_NEAR(stereo_octomap->NearestNeighbor(query), sqrt(1*1 + .5*.5 + 1.5*1.5), TOLERANCE);

    delete stereo_octomap;

}

TEST_F(StereoOctomapTest, CheckAgainstLinearSearch) {

    int num_points = 10000;

    vector<float> x;
    vector<float> y;
    vector<float> z;

    StereoOctomap *stereo_octomap = new StereoOctomap(bot_frames_);

    // create a random point cloud

    std::uniform_real_distribution<double> uniform_dist(-1000, 1000);
    std::random_device rd;
    std::default_random_engine rand_engine(rd());

    for (int i = 0; i < num_points; i++) {

        double this_point[3];

        // generate a random point
        this_point[0] = uniform_dist(rand_engine);
        this_point[1] = uniform_dist(rand_engine);
        this_point[2] = uniform_dist(rand_engine);

        //std::cout << "Point: (" << this_point[0] << ", " << this_point[1] << ", " << this_point[2] << ")" << std::endl;

        double translated_point[3];

        GlobalToCameraFrame(this_point, translated_point);

        x.push_back(translated_point[0]);
        y.push_back(translated_point[1]);
        z.push_back(translated_point[2]);
    }

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    msg.x = x;
    msg.y = y;
    msg.z = z;

    msg.number_of_points = num_points;

    msg.frame_number = 0;
    msg.video_number = 0;

    stereo_octomap->ProcessStereoMessage(&msg);

    // now perform a bunch of searches

    double time_linear = 0, time_octomap = 0;

    int num_searches = 1000;

    for (int i = 0; i < num_searches; i++) {

        double search_point[3];

        // generate a random point
        search_point[0] = uniform_dist(rand_engine);
        search_point[1] = uniform_dist(rand_engine);
        search_point[2] = uniform_dist(rand_engine);

        //std::cout << "search_point: (" << search_point[0] << ", " << search_point[1] << ", " << search_point[2] << ")" << std::endl;


        tic();
        double linear_search_dist = NearestNeighborLinear(x, y, z, search_point);
        time_linear += toc();

        tic();
        double octomap_dist = stereo_octomap->NearestNeighbor(search_point);
        time_octomap += toc();

        EXPECT_NEAR(octomap_dist, linear_search_dist, TOLERANCE);

    }

    std::cout << "\tFor " << num_searches << " searches over " << num_points << " points: " << std:: endl << "\t\tlinear time = " << time_linear << ", octree time = " << time_octomap << ", for a speedup of: " << time_linear / time_octomap << "x" << std::endl;


    delete stereo_octomap;

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

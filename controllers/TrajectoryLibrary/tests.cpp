#include "TrajectoryLibrary.hpp"
#include "Trajectory.hpp"
#include "gtest/gtest.h"
#include "../../utils/utils/RealtimeUtils.hpp"
#include <ctime>
#include <stack>

#define TOLERANCE 0.0001

class TrajectoryLibraryTest : public testing::Test {

    protected:

        virtual void SetUp() {
            // start up lcm

            lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

            param_ = bot_param_new_from_server(lcm_, 0);
            bot_frames_ = bot_frames_get_global(lcm_, param_);

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

        std::stack<clock_t> tictoc_stack;

        void tic() {
            tictoc_stack.push(clock());
        }

        double toc() {
            double outval = ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC;
            tictoc_stack.pop();

            return outval;

        }

        void GlobalToCameraFrame(double point_in[], double point_out[]) {
            // figure out what this point (which is currently expressed in global coordinates
            // will be in local opencv coordinates

            bot_trans_apply_vec(&global_to_camera_trans_, point_in, point_out);
        }

        void CameraToGlobalFrame(double point_in[], double point_out[]) {
            bot_trans_apply_vec(&camera_to_global_trans_, point_in, point_out);
        }

        void AddPointToOctree(StereoOctomap *octomap, double point[]) {
            lcmt_stereo msg;

            msg.timestamp = GetTimestampNow();

            double point_transformed[3];
            GlobalToCameraFrame(point, point_transformed);

            std::cout << "Point: (" << point_transformed[0] << ", " << point_transformed[1] << ", " << point_transformed[2] << ")" << std::endl;

            float x[1], y[1], z[1];
            x[0] = point_transformed[0];
            y[0] = point_transformed[1];
            z[0] = point_transformed[2];

            msg.x = x;
            msg.y = y;
            msg.z = z;

            msg.number_of_points = 1;
            msg.video_number = 0;
            msg.frame_number = 0;

            octomap->ProcessStereoMessage(&msg);
        }



};

/**
 * Loads a trajectory with two points and does basic manipulation of it.
 */
TEST_F(TrajectoryLibraryTest, LoadTrajectory) {
    // load a test trajectory
    Trajectory traj("trajtest/two-point-00000", true);

    // ensure that we can access the right bits

    EXPECT_EQ_ARM(traj.GetDimension(), 12);
    EXPECT_EQ_ARM(traj.GetUDimension(), 3);


    Eigen::VectorXd output = traj.GetState(0);

    Eigen::VectorXd origin(12);
    origin << 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0;


    EXPECT_APPROX_MAT( origin, output, TOLERANCE);


    Eigen::VectorXd point(12);
    point << 1, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0;

    output = traj.GetState(0.01);

    EXPECT_APPROX_MAT(output, point, TOLERANCE);

    Eigen::VectorXd upoint(3);
    upoint << 0, 0, 0;

    EXPECT_APPROX_MAT(upoint, traj.GetUCommand(0), TOLERANCE);

    Eigen::VectorXd upoint2(3);
    upoint2 << 1, 2, 3;

    EXPECT_APPROX_MAT(upoint2, traj.GetUCommand(0.01), TOLERANCE);

    EXPECT_EQ_ARM(traj.GetTimeAtIndex(0), 0);
    EXPECT_EQ_ARM(traj.GetTimeAtIndex(1), 0.01);

    EXPECT_EQ_ARM(traj.GetNumberOfPoints(), 2);

}

/**
 * Tests point transformation with a simple two-point trajectory
 */
TEST_F(TrajectoryLibraryTest, GetTransformedPoint) {
    Trajectory traj("trajtest/two-point-00000", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    trans.trans_vec[0] = 1;

    double point[3] = {1, 0, 0};

    double output[3];

    traj.GetTransformedPoint(0, &trans, output);

    //std::cout << "point = (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    //std::cout << "output = (" << output[0] << ", " << output[1] << ", " << output[2] << ")" << std::endl;

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(point[i], output[i], TOLERANCE);
    }

    trans.trans_vec[0] = 0;
    traj.GetTransformedPoint(1, &trans, output);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(point[i], output[i], TOLERANCE);
    }

    // transform with rotation
    trans.rot_quat[0] = 0.707106781186547;
    trans.rot_quat[1] = 0;
    trans.rot_quat[2] = 0;
    trans.rot_quat[3] = 0.707106781186547;

    traj.GetTransformedPoint(1, &trans, output);

    point[0] = 0;
    point[1] = 1;
    point[2] = 0;

    //std::cout << "point = (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    //std::cout << "output = (" << output[0] << ", " << output[1] << ", " << output[2] << ")" << std::endl;

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(point[i], output[i], TOLERANCE);
    }

}

/**
 * Test FindFurthestTrajectory on:
 *      - no obstacles
 *      - one obstacle
 *      - two obstacles
 */
TEST_F(TrajectoryLibraryTest, FindFurthestTrajectory) {

    StereoOctomap octomap(bot_frames_);

    TrajectoryLibrary lib;

    lib.LoadLibrary("trajtest");

    BotTrans trans;
    bot_trans_set_identity(&trans);

    // check that things work with no obstacles (should return first trajectory)

    double dist;
    Trajectory *best_traj;

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(&octomap, &trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

    EXPECT_TRUE(dist == -1);

    // add an obstacle close to the first trajectory


    double point[3] = { 0.95, 0, 0 };
    AddPointToOctree(&octomap, point);

    std::cout << "test-------------" << std::endl;

    octomap.PrintAllPoints();

    // now we expect to get the second trajectory

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(&octomap, &trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);

    EXPECT_EQ_ARM(dist, 2.0 - 0.95);


}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


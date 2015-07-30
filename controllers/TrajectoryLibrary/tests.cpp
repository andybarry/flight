#include "TrajectoryLibrary.hpp"
#include "Trajectory.hpp"
#include "gtest/gtest.h"
#include "../../utils/utils/RealtimeUtils.hpp"
#include <ctime>
#include <stack>

#define TOLERANCE 0.0001
#define TOLERANCE2 0.001

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

        std::stack<double> tictoc_wall_stack;

        void tic() {
            tictoc_wall_stack.push(GetTimestampNow() / 1000000.0);
        }

        double toc() {
            double outval = GetTimestampNow() / 1000000.0 - tictoc_wall_stack.top();
            tictoc_wall_stack.pop();
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

            float x[1], y[1], z[1];
            x[0] = point[0];
            y[0] = point[1];
            z[0] = point[2];

            AddManyPointsToOctree(octomap, x, y, z, 1);
        }

        void AddManyPointsToOctree(StereoOctomap *octomap, float x_in[], float y_in[], float z_in[], int number_of_points) {
            lcmt_stereo msg;

            msg.timestamp = GetTimestampNow();

            float x[number_of_points];
            float y[number_of_points];
            float z[number_of_points];

            for (int i = 0; i < number_of_points; i++) {

                double this_point[3];

                this_point[0] = x_in[i];
                this_point[1] = y_in[i];
                this_point[2] = z_in[i];

                double point_transformed[3];
                GlobalToCameraFrame(this_point, point_transformed);

                //std::cout << "Point: (" << point_transformed[0] << ", " << point_transformed[1] << ", " << point_transformed[2] << ")" << std::endl;

                x[i] = point_transformed[0];
                y[i] = point_transformed[1];
                z[i] = point_transformed[2];
            }

            msg.x = x;
            msg.y = y;
            msg.z = z;

            msg.number_of_points = number_of_points;
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

TEST_F(TrajectoryLibraryTest, CheckBounds) {
    Trajectory traj("trajtest/two-point-00000", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    double output[3];

    traj.GetTransformedPoint(0, &trans, output);
    EXPECT_EQ_ARM(output[0], 0);

    traj.GetTransformedPoint(0.01, &trans, output);
    EXPECT_EQ_ARM(output[0], 1);

    traj.GetTransformedPoint(10, &trans, output);
    EXPECT_EQ_ARM(output[0], 1);

}

TEST_F(TrajectoryLibraryTest, TestTiRollout) {

    Trajectory traj("trajtest/TI-unit-test-TI-straight-pd-no-yaw-10000", true);

    Eigen::VectorXd expected(12);
    expected << 0,0,0,0,-0.19141,0,12.046,0,-2.3342,0,0,0;

    Eigen::VectorXd output = traj.GetState(0);

    EXPECT_APPROX_MAT(expected, output, TOLERANCE);

    Eigen::VectorXd expected2(12);
    expected2 << 26.135,0,9.2492e-09,0,-0.19141,0,12.046,0,-2.3342,0,7.8801e-13,0;

    output = traj.GetRolloutState(2.13);

    EXPECT_APPROX_MAT( expected2, output, TOLERANCE);



}


TEST_F(TrajectoryLibraryTest, LoadLibrary) {
    TrajectoryLibrary lib;

    ASSERT_TRUE(lib.LoadLibrary("trajtest", true));

    EXPECT_EQ_ARM(lib.GetNumberTVTrajectories(), 2);

    EXPECT_EQ_ARM(lib.GetNumberStableTrajectories(), 1);

    EXPECT_EQ_ARM(lib.GetTrajectoryByNumber(10000)->GetTrajectoryNumber(), 10000);
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

    lib.LoadLibrary("trajtest", true);

    EXPECT_EQ_ARM(lib.GetNumberTVTrajectories(), 2);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    // check that things work with no obstacles (should return first trajectory)

    double dist;
    const Trajectory *best_traj;

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

    EXPECT_TRUE(dist == -1);

    // add an obstacle close to the first trajectory


    double point[3] = { 1.05, 0, 0 };
    AddPointToOctree(&octomap, point);

    // now we expect to get the second trajectory

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);

    EXPECT_NEAR(dist, 2.0 - 1.05, TOLERANCE);

    // add another point close to the second trajectory

    point[0] = 2.01;
    point[1] = 0.01;
    point[2] = -0.015;

    AddPointToOctree(&octomap, point);


    // now we expect to get the first trajectory

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

    EXPECT_NEAR(dist, 0.05, TOLERANCE);

}

TEST_F(TrajectoryLibraryTest, TwoTrajectoriesOnePointWithTransform) {

    StereoOctomap octomap(bot_frames_);

    TrajectoryLibrary lib;

    lib.LoadLibrary("trajtest", true);

    double point[3] = { 1.05, 0, 0 };
    AddPointToOctree(&octomap, point);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    double dist;
    const Trajectory *best_traj;
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);

    trans.rot_quat[0] = 0.8349;
    trans.rot_quat[1] = 0.3300;
    trans.rot_quat[2] = 0.4236;
    trans.rot_quat[3] = -0.1206;

    // rotation shouldn't matter
    // TODO: this will fail when I introduce aircraft rotation into the check

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);
    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);

    // with a translation, we expect a different result

    trans.trans_vec[0] = 1;

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);
    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

}

TEST_F(TrajectoryLibraryTest, ManyPointsAgainstMatlab) {
    StereoOctomap octomap(bot_frames_);

    // load points
    float x[30] = {10.6599, 22.2863, 17.7719, 25.3705, 11.2680, 8.5819, 18.7618, 15.5065, 30.2579, 23.4038, 31.0982, 8.5764, 19.3839, 24.4193, 20.6749, 8.1739, 17.1374, 27.9051, 32.9856, 9.1662, 20.7621, 19.5456, 27.2377, 9.4999, 6.3336, 18.2721, 27.0902, 26.1214, 14.9257, 10.9116};

    float y[30] = {-6.3751, 5.5009, 4.3333, 4.0736, 6.2785, 3.2191, 4.8583, 4.8603, 9.9875, 2.4675, -7.0566, 13.1949, 4.1795, 1.3166, 14.8111, -11.7091, -1.5488, 3.8369, 14.1822, 5.8880, 0.9103, -3.1963, 0.6016, 2.5828, 7.6480, 5.6339, -3.1588, -1.7308, -2.2707, 9.6516};

    float z[30] = {-12.2666, 1.3978, 4.4285, 13.3552, -7.9131, -1.4959, 8.1086, -2.5152, -7.3068, 1.2222, -5.4578, 4.3666, 1.3415, 6.6314, -8.4397, -13.0923, -4.0255, 8.1594, -9.2391, -12.1854, 10.8342, 5.1429, -4.5686, -7.1356, -7.7164, -4.2232, 5.5025, -14.4127, -6.8919, -2.1024};

    AddManyPointsToOctree(&octomap, x, y, z, 30);

    TrajectoryLibrary lib;

    lib.LoadLibrary("trajtest-many", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    double dist;
    const Trajectory *best_traj;
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 50.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 1);

    EXPECT_NEAR(dist, 4.2911, 0.001);


}

TEST_F(TrajectoryLibraryTest, TimingTest) {
    StereoOctomap octomap(bot_frames_);

    // create a random point cloud

    int num_points = 10000;
    int num_lookups = 1000;

    std::uniform_real_distribution<double> uniform_dist(-1000, 1000);
    std::random_device rd;
    std::default_random_engine rand_engine(rd());

    float x[num_points], y[num_points], z[num_points];

    for (int i = 0; i < num_points; i++) {


        // generate a random point
        x[i] = uniform_dist(rand_engine);
        y[i] = uniform_dist(rand_engine);
        z[i] = uniform_dist(rand_engine);

        //std::cout << "Point: (" << this_point[0] << ", " << this_point[1] << ", " << this_point[2] << ")" << std::endl;

    }

    AddManyPointsToOctree(&octomap, x, y, z, num_points);

    TrajectoryLibrary lib;

    lib.LoadLibrary("trajtest-many", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    tic();

    for (int i = 0; i < num_lookups; i++) {
        double dist;
        const Trajectory *best_traj;
        std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 50.0);
    }

    double num_sec = toc();

    std::cout << num_lookups <<  " lookups with " << lib.GetNumberTVTrajectories() << " TV trajectories on a cloud (" << num_points << ") took: " << num_sec << " sec (" << num_sec / (double)num_lookups*1000.0 << " ms / lookup " << std::endl;
}

TEST_F(TrajectoryLibraryTest, RemainderTrajectorySimple) {
    StereoOctomap octomap(bot_frames_);

    // get a trajectory

    TrajectoryLibrary lib;
    lib.LoadLibrary("trajtest", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    const Trajectory *traj = lib.GetTrajectoryByNumber(0);

    double dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0);

    // with no obstacles, we expect -1
    EXPECT_EQ_ARM(dist, -1);

    // add an obstacle
    double point[3] = { 0, 0, 0};
    AddPointToOctree(&octomap, point);

    // now we expect zero distance since the obstacle and the trajectory start at the origin
    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0);
    EXPECT_EQ_ARM(dist, 0);

    // at t = 0.01, we expect a distance of 1 since the point is already behind us
    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0.01);
    EXPECT_EQ_ARM(dist, 1);

    // add a transform
    trans.trans_vec[2] = 42;

    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0);
    EXPECT_EQ_ARM(dist, 42);

    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0.01);
    EXPECT_NEAR(dist, sqrt( 42*42 + 1*1  ), TOLERANCE);
}

TEST_F(TrajectoryLibraryTest, RemainderTrajectory) {
    StereoOctomap octomap(bot_frames_);



    // Load a complicated trajectory
    TrajectoryLibrary lib;
    lib.LoadLibrary("trajtest-many", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    const Trajectory *traj = lib.GetTrajectoryByNumber(1);

    // with no obstacles, we expect -1
//    double dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0);
//    EXPECT_EQ_ARM(dist, -1);

double dist;
    double point[3] = { 6.65, -7.23, 9.10 };
    AddPointToOctree(&octomap, point);
    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0);
    EXPECT_NEAR(dist, 11.748141101535500, TOLERANCE2); // from matlab

    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 0.95);
    EXPECT_NEAR(dist, 11.8831, TOLERANCE2);

    dist = lib.ClosestObstacleInRemainderOfTrajectory(octomap, trans, *traj, 1.5); // t after trajectory
    EXPECT_NEAR(dist, 12.3832, TOLERANCE2); // from matlab

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


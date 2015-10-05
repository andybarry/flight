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
            bot_frames_ = bot_frames_new(lcm_, param_);

            bot_frames_get_trans(bot_frames_, "local", "opencvFrame", &global_to_camera_trans_);
            bot_frames_get_trans(bot_frames_, "opencvFrame", "local", &camera_to_global_trans_);

            lcmgl_ = bot_lcmgl_init(lcm_, "TrajectoryLibraryTest");

        }

        virtual void TearDown() {
            bot_lcmgl_destroy(lcmgl_);
            lcm_destroy(lcm_);
            // todo: delete param_;
        }


        lcm_t *lcm_;
        BotParam *param_;
        BotFrames *bot_frames_;
        BotTrans global_to_camera_trans_, camera_to_global_trans_;
        bot_lcmgl_t *lcmgl_;

        std::stack<double> tictoc_wall_stack;


        // these are some random points to use for some of the tests
        float x_points_[30] = {10.6599, 22.2863, 17.7719, 25.3705, 11.2680, 8.5819, 18.7618, 15.5065, 30.2579, 23.4038, 31.0982, 8.5764, 19.3839, 24.4193, 20.6749, 8.1739, 17.1374, 27.9051, 32.9856, 9.1662, 20.7621, 19.5456, 27.2377, 9.4999, 6.3336, 18.2721, 27.0902, 26.1214, 14.9257, 10.9116};

        float y_points_[30] = {-6.3751, 5.5009, 4.3333, 4.0736, 6.2785, 3.2191, 4.8583, 4.8603, 9.9875, 2.4675, -7.0566, 13.1949, 4.1795, 1.3166, 14.8111, -11.7091, -1.5488, 3.8369, 14.1822, 5.8880, 0.9103, -3.1963, 0.6016, 2.5828, 7.6480, 5.6339, -3.1588, -1.7308, -2.2707, 9.6516};

        float z_points_[30] = {-12.2666, 1.3978, 4.4285, 13.3552, -7.9131, -1.4959, 8.1086, -2.5152, -7.3068, 1.2222, -5.4578, 4.3666, 1.3415, 6.6314, -8.4397, -13.0923, -4.0255, 8.1594, -9.2391, -12.1854, 10.8342, 5.1429, -4.5686, -7.1356, -7.7164, -4.2232, 5.5025, -14.4127, -6.8919, -2.1024};

        int number_of_reference_points_ = 30;

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

        void AddPointToOctree(StereoOctomap *octomap, double point[], double altitude_offset) {

            float x[1], y[1], z[1];
            x[0] = point[0];
            y[0] = point[1];
            z[0] = point[2];

            AddManyPointsToOctree(octomap, x, y, z, 1, altitude_offset);
        }

        void AddManyPointsToOctree(StereoOctomap *octomap, float x_in[], float y_in[], float z_in[], int number_of_points, double altitude_offset) {
            lcmt::stereo msg;

            msg.timestamp = GetTimestampNow();

            vector<float> x, y, z;

            for (int i = 0; i < number_of_points; i++) {

                double this_point[3];

                this_point[0] = x_in[i];
                this_point[1] = y_in[i];
                this_point[2] = z_in[i] + altitude_offset;

                double point_transformed[3];
                GlobalToCameraFrame(this_point, point_transformed);

                //std::cout << "Point: (" << point_transformed[0] << ", " << point_transformed[1] << ", " << point_transformed[2] << ")" << std::endl;

                x.push_back(point_transformed[0]);
                y.push_back(point_transformed[1]);
                z.push_back(point_transformed[2]);
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
    Trajectory traj("trajtest/simple/two-point-00000", true);

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

TEST_F(TrajectoryLibraryTest, MinimumAltitude) {
    Trajectory traj("trajtest/simple/two-point-00000", true);
    EXPECT_EQ_ARM(traj.GetMinimumAltitude(), 0);


    Trajectory traj2("trajtest/full/unit-testing-super-aggressive-dive-open-loop-00009", true);
    EXPECT_EQ_ARM(traj2.GetMinimumAltitude(), -7.9945);

    Trajectory traj3("trajtest/full/unit-testing-knife-edge-open-loop-00010", true);
    EXPECT_EQ_ARM(traj3.GetMinimumAltitude(), -1.2897);
}

/**
 * Tests point transformation with a simple two-point trajectory
 */
TEST_F(TrajectoryLibraryTest, GetTransformedPoint) {
    Trajectory traj("trajtest/simple/two-point-00000", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    trans.trans_vec[0] = 1;

    double point[3] = {1, 0, 0};

    double output[3];

    traj.GetXyzYawTransformedPoint(0, trans, output);

    //std::cout << "point = (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
    //std::cout << "output = (" << output[0] << ", " << output[1] << ", " << output[2] << ")" << std::endl;

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(point[i], output[i], TOLERANCE);
    }

    trans.trans_vec[0] = 0;
    traj.GetXyzYawTransformedPoint(1, trans, output);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(point[i], output[i], TOLERANCE);
    }

    // transform with rotation
    trans.rot_quat[0] = 0.707106781186547;
    trans.rot_quat[1] = 0;
    trans.rot_quat[2] = 0;
    trans.rot_quat[3] = 0.707106781186547;

    traj.GetXyzYawTransformedPoint(1, trans, output);

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
    Trajectory traj("trajtest/simple/two-point-00000", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);

    double output[3];

    traj.GetXyzYawTransformedPoint(0, trans, output);
    EXPECT_EQ_ARM(output[0], 0);

    traj.GetXyzYawTransformedPoint(0.01, trans, output);
    EXPECT_EQ_ARM(output[0], 1);

    traj.GetXyzYawTransformedPoint(10, trans, output);
    EXPECT_EQ_ARM(output[0], 1);

}

TEST_F(TrajectoryLibraryTest, TestTiRollout) {

    Trajectory traj("trajtest/ti/TI-test-TI-straight-pd-no-yaw-00000", true);

    Eigen::VectorXd expected(12);
    expected << 0,0,0,0,-0.19141,0,12.046,0,-2.3342,0,0,0;

    Eigen::VectorXd output = traj.GetState(0);

    EXPECT_APPROX_MAT(expected, output, TOLERANCE);

    Eigen::VectorXd expected2(12);
    expected2 << 26.135,0,9.2492e-09,0,-0.19141,0,12.046,0,-2.3342,0,7.8801e-13,0;

    output = traj.GetState(2.13);

    EXPECT_APPROX_MAT( expected2, output, TOLERANCE);



}


TEST_F(TrajectoryLibraryTest, LoadLibrary) {
    TrajectoryLibrary lib(0);

    ASSERT_TRUE(lib.LoadLibrary("trajtest/simple", true));

    EXPECT_EQ_ARM(lib.GetNumberTrajectories(), 2);

    EXPECT_EQ_ARM(lib.GetTrajectoryByNumber(0)->GetTrajectoryNumber(), 0);
}

/**
 * Test FindFarthestTrajectory on:
 *      - no obstacles
 *      - one obstacle
 *      - two obstacles
 */
TEST_F(TrajectoryLibraryTest, FindFarthestTrajectory) {

    StereoOctomap octomap(bot_frames_);

    TrajectoryLibrary lib(0);

    lib.LoadLibrary("trajtest/simple", true);

    EXPECT_EQ_ARM(lib.GetNumberTrajectories(), 2);

    BotTrans trans;
    bot_trans_set_identity(&trans);
    double altitude = 30;
    trans.trans_vec[2] = altitude; // flying high up

    // check that things work with no obstacles (should return first trajectory)

    double dist;
    const Trajectory *best_traj;

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

    // check for preferred trajectory
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0, nullptr, 1);
    ASSERT_TRUE(best_traj != nullptr);
    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);

    // preferred trajectory error
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0, nullptr, 2);
    ASSERT_TRUE(best_traj != nullptr);
    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

    // add an obstacle close to the first trajectory
    double point[3] = { 1.05, 0, 0 };
    AddPointToOctree(&octomap, point, altitude);

    // now we expect to get the second trajectory

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);

    EXPECT_NEAR(dist, 2.0 - 1.05, TOLERANCE);

    // add another point close to the second trajectory

    point[0] = 2.01;
    point[1] = 0.01;
    point[2] = -0.015;

    AddPointToOctree(&octomap, point, altitude);



    // now we expect to get the first trajectory

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 0);

    EXPECT_NEAR(dist, 0.05, TOLERANCE);

}

TEST_F(TrajectoryLibraryTest, TwoTrajectoriesOnePointWithTransform) {

    StereoOctomap octomap(bot_frames_);

    TrajectoryLibrary lib(0);

    lib.LoadLibrary("trajtest/simple", true);
    double altitude = 30;

    double point[3] = { 1.05, 0, 0 };
    AddPointToOctree(&octomap, point, altitude);

    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

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

TEST_F(TrajectoryLibraryTest, Altitude) {
    StereoOctomap octomap(bot_frames_);
    TrajectoryLibrary lib(0);

    lib.LoadLibrary("trajtest/simple", true);

    double altitude = 0.5;
    double point[3] = { 1.05, 0, 0 };
    AddPointToOctree(&octomap, point, altitude);

    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    double dist;
    const Trajectory *best_traj;
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 2.0);

    EXPECT_TRUE(best_traj->GetTrajectoryNumber() == 1);
}

TEST_F(TrajectoryLibraryTest, ManyTrajectoriesWithTransform) {
    StereoOctomap octomap(bot_frames_);

    TrajectoryLibrary lib(0);
    lib.LoadLibrary("trajtest/full", true);

    double altitude = 30;

    double point[3] = {18, 12, 0};
    AddPointToOctree(&octomap, point, altitude);

    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[0] = 17;
    trans.trans_vec[1] = 11;
    trans.trans_vec[2] = altitude;

    // send th

    double dist;
    const Trajectory *best_traj;

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 5.0);

    DrawOriginLcmGl(lcm_);

    bot_lcmgl_color3f(lcmgl_, 0, 0, 1);
    best_traj->Draw(lcmgl_, &trans);


    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 3);
    EXPECT_NEAR(dist, 1.025243, TOLERANCE);

    // now add a yaw
    trans.rot_quat[0] = 0.642787609686539;
    trans.rot_quat[1] = 0;
    trans.rot_quat[2] = 0;
    trans.rot_quat[3] = 0.766044443118978;

    bot_lcmgl_color3f(lcmgl_, 1, 0, 0);
    best_traj->Draw(lcmgl_, &trans);
    bot_lcmgl_switch_buffer(lcmgl_);

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 5.0);
    //lib.Draw(lcm_, &trans);

    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 2);
    EXPECT_NEAR(dist, 1.174604, TOLERANCE);

    // now have a transform with roll, pitch, and yaw
    trans.rot_quat[0] = 0.863589399067779;
    trans.rot_quat[1] = -0.004581450790098;
    trans.rot_quat[2] = 0.298930259006064;
    trans.rot_quat[3] = 0.405996379758463;

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 5.0);

    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 4);
    EXPECT_NEAR(dist, 0.327772, TOLERANCE);
}

TEST_F(TrajectoryLibraryTest, ManyPointsAgainstMatlab) {
    StereoOctomap octomap(bot_frames_);
    double altitude = 30;

    // load points
    AddManyPointsToOctree(&octomap, x_points_, y_points_, z_points_, number_of_reference_points_, altitude);

    TrajectoryLibrary lib(0);

    lib.LoadLibrary("trajtest/many", true);


    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    double dist;
    const Trajectory *best_traj;
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 50.0);

    ASSERT_TRUE(best_traj != nullptr);

    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 1);

    EXPECT_NEAR(dist, 4.2911, 0.001);


}

TEST_F(TrajectoryLibraryTest, TimingTest) {
    StereoOctomap octomap(bot_frames_);

    double altitude = 30;

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

    AddManyPointsToOctree(&octomap, x, y, z, num_points, altitude);

    TrajectoryLibrary lib(0);

    lib.LoadLibrary("trajtest/many", true);

    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    tic();

    for (int i = 0; i < num_lookups; i++) {
        double dist;
        const Trajectory *best_traj;
        std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 50.0);
    }

    double num_sec = toc();

    std::cout << num_lookups <<  " lookups with " << lib.GetNumberTrajectories() << " trajectories on a cloud (" << num_points << ") took: " << num_sec << " sec (" << num_sec / (double)num_lookups*1000.0 << " ms / lookup)" << std::endl;
}

TEST_F(TrajectoryLibraryTest, RemainderTrajectorySimple) {
    StereoOctomap octomap(bot_frames_);

    // get a trajectory

    TrajectoryLibrary lib(0);
    lib.LoadLibrary("trajtest/simple", true);

    double altitude = 100;

    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    const Trajectory *traj = lib.GetTrajectoryByNumber(0);

    double dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);

    // add an obstacle
    double point[3] = { 0, 0, 0};
    AddPointToOctree(&octomap, point, altitude);

    // now we expect zero distance since the obstacle and the trajectory start at the origin
    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);
    EXPECT_NEAR(dist, 0, TOLERANCE);

    // at t = 0.01, we expect a distance of 1 since the point is already behind us
    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0.01, 0);
    EXPECT_EQ_ARM(dist, 1);

    // add a transform
    trans.trans_vec[2] += 42;

    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);
    EXPECT_EQ_ARM(dist, 42);

    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0.01, 0);
    EXPECT_NEAR(dist, sqrt( 42*42 + 1*1  ), TOLERANCE);
}

TEST_F(TrajectoryLibraryTest, RemainderTrajectorySimpleAltitude) {
    StereoOctomap octomap(bot_frames_);

    // get a trajectory
    TrajectoryLibrary lib(0);
    lib.LoadLibrary("trajtest/simple", true);

    double altitude = 5;

    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    const Trajectory *traj = lib.GetTrajectoryByNumber(0);
    double dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);


    EXPECT_EQ_ARM(dist, -1);

    // add an obstacle
    double point[3] = { 0, 0, 6};
    AddPointToOctree(&octomap, point, 0);

    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);

    EXPECT_EQ_ARM(dist, 6 - altitude);
}

TEST_F(TrajectoryLibraryTest, RemainderTrajectory) {
    StereoOctomap octomap(bot_frames_);

    // Load a complicated trajectory
    TrajectoryLibrary lib(0);
    lib.LoadLibrary("trajtest/many", true);

    double altitude = 30;
    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    const Trajectory *traj = lib.GetTrajectoryByNumber(1);

    double dist;

    double point[3] = { 6.65, -7.23, 9.10 };
    AddPointToOctree(&octomap, point, altitude);
    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);
    EXPECT_NEAR(dist, 11.748141101535500, TOLERANCE2); // from matlab

    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0.95, 0);
    EXPECT_NEAR(dist, 11.8831, TOLERANCE2);

    dist = traj->ClosestObstacleInRemainderOfTrajectory(octomap, trans, 1.5, 0); // t after trajectory
    EXPECT_NEAR(dist, 12.3832, TOLERANCE2); // from matlab
}

TEST_F(TrajectoryLibraryTest, RemainderTrajectoryTi) {
    StereoOctomap octomap(bot_frames_);

    double altitude = 30;
    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    Trajectory traj("trajtest/ti/TI-test-TI-straight-pd-no-yaw-00000", true);

    double dist = traj.ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);
    EXPECT_EQ_ARM(dist, -1);

    double point[3] = { 1.5, -0.5, 3 };
    AddPointToOctree(&octomap, point, altitude);
    dist = traj.ClosestObstacleInRemainderOfTrajectory(octomap, trans, 0, 0);
    EXPECT_NEAR(dist, 3.041506, TOLERANCE2); // from matlab

    dist = traj.ClosestObstacleInRemainderOfTrajectory(octomap, trans, 1.21, 0);
    EXPECT_NEAR(dist, 13.6886, TOLERANCE2);

}

TEST_F(TrajectoryLibraryTest, FindFarthestWithTI) {
    StereoOctomap octomap(bot_frames_);

    TrajectoryLibrary lib(0);
    lib.LoadLibrary("trajtest/full", true);

    double altitude = 30;
    BotTrans trans;
    bot_trans_set_identity(&trans);
    trans.trans_vec[2] = altitude;

    double dist;
    const Trajectory *best_traj;
    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 50.0);

    ASSERT_TRUE(best_traj != nullptr);
    // with no obstacles, we should always get the first trajectory
    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 0);

    // put an obstacle along the path of the TI trajectory
    double point[3] = { 1.10, -0.1, 0 };
    AddPointToOctree(&octomap, point, altitude);

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap, trans, 50.0);

    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 4); // from matlab
    EXPECT_NEAR(dist, 0.2032, TOLERANCE2); // from matlab

    // create a new octre
    StereoOctomap octomap2(bot_frames_);

    AddManyPointsToOctree(&octomap2, x_points_, y_points_, z_points_, number_of_reference_points_, altitude);

    std::tie(dist, best_traj) = lib.FindFarthestTrajectory(octomap2, trans, 50.0);

    EXPECT_EQ_ARM(best_traj->GetTrajectoryNumber(), 3);
    EXPECT_NEAR(dist, 5.0060, TOLERANCE);


}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


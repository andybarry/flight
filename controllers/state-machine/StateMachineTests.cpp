#include "StateMachineControl.hpp"
#include "gtest/gtest.h"
#include "../../utils/utils/RealtimeUtils.hpp"
#include <ctime>
#include <stack>

#define TOLERANCE 0.0001
#define TOLERANCE2 0.001

class StateMachineControlTest : public testing::Test {

    protected:

        virtual void SetUp() {
            lcm_ = new lcm::LCM("udpm://239.255.76.67:7667?ttl=0");

            if (!lcm_->good()) {
                std::cerr << "LCM creation failed." << std::endl;
                exit(1);
            }

            // get parameter server
            BotParam *param = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
            bot_frames_ = bot_frames_get_global(lcm_->getUnderlyingLCM(), param);

            bot_frames_get_trans(bot_frames_, "local", "opencvFrame", &global_to_camera_trans_);
            bot_frames_get_trans(bot_frames_, "opencvFrame", "local", &camera_to_global_trans_);

        }

        virtual void TearDown() {
            delete lcm_;
            // todo: delete param_;
        }


        lcm::LCM *lcm_;
        BotParam *param_;
        BotFrames *bot_frames_;
        BotTrans global_to_camera_trans_, camera_to_global_trans_;

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

        void AddPointToOctree(StereoOctomap *octomap, double point[]) {

           // float x[1], y[1], z[1];
            //x[0] = point[0];
            //y[0] = point[1];
            //z[0] = point[2];

            //AddManyPointsToOctree(octomap, x, y, z, 1);
        }

        void AddManyPointsToOctree(StereoOctomap *octomap, float x_in[], float y_in[], float z_in[], int number_of_points) {
            /*
             * lcmt_stereo msg;

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

            //octomap->ProcessStereoMessage(&msg);
            */
        }



};

TEST_F(StateMachineControlTest, BasicStateMachineTest) {
    int stable_traj_num = 0;

    StateMachineControl *fsm_control = new StateMachineControl(lcm_, "../TrajectoryLibrary/trajtest/simple", bot_frames_, 5.0, stable_traj_num, "tvlqr-action-out");

    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), stable_traj_num);

    EXPECT_EQ_ARM(fsm_control->GetStableTrajectory().GetTrajectoryNumber(), stable_traj_num);

    delete fsm_control;
}

TEST_F(StateMachineControlTest, NoObstaclesStaysInCruise) {
    int stable_traj_num = 0;

    StateMachineControl *fsm_control = new StateMachineControl(lcm_, "../TrajectoryLibrary/trajtest/simple", bot_frames_, 5.0, stable_traj_num, "tvlqr-action-out");

    // ensure that we are in the start state
    EXPECT_TRUE(fsm_control->GetCurrentStateName().compare("AirplaneFsm::Cruise") == 0);

    delete fsm_control;
}





int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "StateMachineControl*";
  return RUN_ALL_TESTS();
}


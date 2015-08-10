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
            bot_frames_ = bot_frames_new(lcm_->getUnderlyingLCM(), param);

            bot_frames_get_trans(bot_frames_, "local", "opencvFrame", &global_to_camera_trans_);
            bot_frames_get_trans(bot_frames_, "opencvFrame", "local", &camera_to_global_trans_);

        }

        virtual void TearDown() {
            delete lcm_;
            // todo: delete param_;
        }

        std::string pose_channel_ = "STATE_ESTIMATOR_POSE";
        std::string stereo_channel_ = "stereo";


        lcm::LCM *lcm_;
        BotFrames *bot_frames_;
        BotTrans global_to_camera_trans_, camera_to_global_trans_;

        std::stack<double> tictoc_wall_stack;


        // these are some random points to use for some of the tests
        vector<float> x_points_ = {10.6599, 22.2863, 17.7719, 25.3705, 11.2680, 8.5819, 18.7618, 15.5065, 30.2579, 23.4038, 31.0982, 8.5764, 19.3839, 24.4193, 20.6749, 8.1739, 17.1374, 27.9051, 32.9856, 9.1662, 20.7621, 19.5456, 27.2377, 9.4999, 6.3336, 18.2721, 27.0902, 26.1214, 14.9257, 10.9116};

        vector<float> y_points_ = {-6.3751, 5.5009, 4.3333, 4.0736, 6.2785, 3.2191, 4.8583, 4.8603, 9.9875, 2.4675, -7.0566, 13.1949, 4.1795, 1.3166, 14.8111, -11.7091, -1.5488, 3.8369, 14.1822, 5.8880, 0.9103, -3.1963, 0.6016, 2.5828, 7.6480, 5.6339, -3.1588, -1.7308, -2.2707, 9.6516};

        vector<float> z_points_ = {-12.2666, 1.3978, 4.4285, 13.3552, -7.9131, -1.4959, 8.1086, -2.5152, -7.3068, 1.2222, -5.4578, 4.3666, 1.3415, 6.6314, -8.4397, -13.0923, -4.0255, 8.1594, -9.2391, -12.1854, 10.8342, 5.1429, -4.5686, -7.1356, -7.7164, -4.2232, 5.5025, -14.4127, -6.8919, -2.1024};

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

        void SendStereoPoint(float point[]) {

            vector<float> x, y, z;
            x.push_back(point[0]);
            y.push_back(point[1]);
            z.push_back(point[2]);

            SendStereoManyPoints(x, y, z);
        }

        void SendStereoManyPoints(vector<float> x_in, vector<float> y_in, vector<float> z_in) {

            lcmt::stereo msg;

            msg.timestamp = GetTimestampNow();

            vector<float> x, y, z;
            vector<unsigned char> grey;

            for (int i = 0; i < (int)x_in.size(); i++) {

                double this_point[3];

                this_point[0] = x_in[i];
                this_point[1] = y_in[i];
                this_point[2] = z_in[i];

                double point_transformed[3];
                GlobalToCameraFrame(this_point, point_transformed);

                ////std::cout << "Point: (" << point_transformed[0] << ", " << point_transformed[1] << ", " << point_transformed[2] << ")" << std::endl;

                x.push_back(point_transformed[0]);
                y.push_back(point_transformed[1]);
                z.push_back(point_transformed[2]);

                grey.push_back(0);
            }

            msg.x = x;
            msg.y = y;
            msg.z = z;

            msg.grey = grey;

            msg.number_of_points = x.size();
            msg.video_number = 0;
            msg.frame_number = 0;

            lcm_->publish("stereo", &msg);

        }

        void ProcessAllLcmMessages() {
            while (NonBlockingLcm(lcm_->getUnderlyingLCM())) {}
        }

        mav::pose_t GetDefaultPoseMsg() {
            mav::pose_t msg;

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

            return msg;
        }



};

TEST_F(StateMachineControlTest, BasicStateMachineTest) {
    StateMachineControl *fsm_control = new StateMachineControl(lcm_, "../TrajectoryLibrary/trajtest/full", "tvlqr-action-out");

    int stable_traj_num = 0;

    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), stable_traj_num);

    EXPECT_EQ_ARM(fsm_control->GetStableTrajectory().GetTrajectoryNumber(), stable_traj_num);

    delete fsm_control;
}

TEST_F(StateMachineControlTest, OneObstacle) {
    StateMachineControl *fsm_control = new StateMachineControl(lcm_, "../TrajectoryLibrary/trajtest/full", "tvlqr-action-out");

    // subscribe to LCM channels
    lcm::Subscription *pose_sub = lcm_->subscribe(pose_channel_, &StateMachineControl::ProcessImuMsg, fsm_control);
    lcm::Subscription *stereo_sub = lcm_->subscribe(stereo_channel_, &StateMachineControl::ProcessStereoMsg, fsm_control);

    //fsm_control->GetFsmContext()->setDebugFlag(true);

    // ensure that we are in the start state
    EXPECT_TRUE(fsm_control->GetCurrentStateName().compare("AirplaneFsm::ExecuteTrajectory") == 0);

    // send some pose messages
    mav::pose_t msg = GetDefaultPoseMsg();

    // check for unexpected transitions out of Cruise state

    lcm_->publish(pose_channel_, &msg);

    ProcessAllLcmMessages();

    EXPECT_TRUE(fsm_control->GetCurrentStateName().compare("AirplaneFsm::ExecuteTrajectory") == 0);
    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 0);

    lcm_->publish(pose_channel_, &msg);
    lcm_->publish(pose_channel_, &msg);
    lcm_->publish(pose_channel_, &msg);
    lcm_->publish(pose_channel_, &msg);
    ProcessAllLcmMessages();

    EXPECT_TRUE(fsm_control->GetCurrentStateName().compare("AirplaneFsm::ExecuteTrajectory") == 0);
    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 0);

    // now add an obstacle in front of it and make sure it transitions!
    float point[3] = { 24, 0, 0 };
    SendStereoPoint(point);

    lcm_->publish(pose_channel_, &msg);
    ProcessAllLcmMessages();

    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 2); // from matlab, with 5.0 = safe

    // NOTE: modification of x_points_ if you're checking in MATLAB (!)
    for (int i = 0; i < int(x_points_.size()); i++) {
        x_points_[i] += 5;
    }

    SendStereoManyPoints(x_points_, y_points_, z_points_);
    lcm_->publish(pose_channel_, &msg);
    ProcessAllLcmMessages();

    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 3); // from matlab

    delete fsm_control;

    lcm_->unsubscribe(pose_sub);
    lcm_->unsubscribe(stereo_sub);
}

/**
 * Tests that the state machine requests the cruise trajectory after a timeout
 */
TEST_F(StateMachineControlTest, TrajectoryTimeout) {

    StateMachineControl *fsm_control = new StateMachineControl(lcm_, "../TrajectoryLibrary/trajtest/full", "tvlqr-action-out");

    // subscribe to LCM channels
    lcm::Subscription *pose_sub = lcm_->subscribe(pose_channel_, &StateMachineControl::ProcessImuMsg, fsm_control);
    lcm::Subscription *stereo_sub = lcm_->subscribe(stereo_channel_, &StateMachineControl::ProcessStereoMsg, fsm_control);

    // send an obstacle to get it to transition to a new time

    float point[3] = { 17, 11, 3 };
    SendStereoPoint(point);

    float point2[3] = { 24, 0, 0 };
    SendStereoPoint(point2);

    mav::pose_t msg = GetDefaultPoseMsg();

    lcm_->publish(pose_channel_, &msg);
    ProcessAllLcmMessages();

    // ensure that we have changed trajectories
    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 3); // from matlab

    // wait for that trajectory to time out
    int64_t t_start = GetTimestampNow();
    double t = 0;
    while (t < 3.1) {
        usleep(7142); // 1/140 of a second

        msg.utime = GetTimestampNow();

        t = (msg.utime - t_start) / 1000000.0;

        msg.pos[0] = t * 10;

        lcm_->publish(pose_channel_, &msg);
        ProcessAllLcmMessages();
    }

    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 0);


    delete fsm_control;

    lcm_->unsubscribe(pose_sub);
    lcm_->unsubscribe(stereo_sub);
}


/**
 * Tests an obstacle appearing during a trajectory execution
 */
TEST_F(StateMachineControlTest, TrajectoryInterrupt) {

    StateMachineControl *fsm_control = new StateMachineControl(lcm_, "../TrajectoryLibrary/trajtest/full", "tvlqr-action-out");
    fsm_control->GetFsmContext()->setDebugFlag(true);

    // subscribe to LCM channels
    lcm::Subscription *pose_sub = lcm_->subscribe(pose_channel_, &StateMachineControl::ProcessImuMsg, fsm_control);
    lcm::Subscription *stereo_sub = lcm_->subscribe(stereo_channel_, &StateMachineControl::ProcessStereoMsg, fsm_control);

    // send an obstacle to get it to transition to a new time

    float point[3] = { 24, 0, 0 };
    SendStereoPoint(point);

    mav::pose_t msg = GetDefaultPoseMsg();

    lcm_->publish(pose_channel_, &msg);
    ProcessAllLcmMessages();

    // ensure that we have changed trajectories
    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 2); // from matlab

    // wait for that trajectory to time out
    int64_t t_start = GetTimestampNow();
    double t = 0;
    while (t < 2.0) {
        usleep(7142); // 1/140 of a second

        msg.utime = GetTimestampNow();

        t = (msg.utime - t_start) / 1000000.0;

        Eigen::VectorXd state_t = fsm_control->GetCurrentTrajectory().GetState(t);

        msg.pos[0] = state_t(0);
        msg.pos[1] = state_t(1);
        msg.pos[2] = state_t(2);

        msg.vel[0] = state_t(6);
        msg.vel[1] = state_t(7);
        msg.vel[2] = state_t(8);

        double rpy[3];
        rpy[0] = state_t(3);
        rpy[1] = state_t(4);
        rpy[2] = state_t(5);

        double quat[4];
        bot_roll_pitch_yaw_to_quat(rpy, quat);

        msg.orientation[0] = quat[0];
        msg.orientation[1] = quat[1];
        msg.orientation[2] = quat[2];
        msg.orientation[3] = quat[3];

        msg.rotation_rate[0] = state_t(9);
        msg.rotation_rate[1] = state_t(10);
        msg.rotation_rate[2] = state_t(11);


        lcm_->publish(pose_channel_, &msg);
        ProcessAllLcmMessages();
    }

    // now add a new obstacle right in front!
    std::cout << "NEW POINT" << std::endl;
    float point2[3] = { 18, 12, 0 };
    SendStereoPoint(point2);
    lcm_->publish(pose_channel_, &msg);
    ProcessAllLcmMessages();

    EXPECT_EQ_ARM(fsm_control->GetCurrentTrajectory().GetTrajectoryNumber(), 4); // from matlab


    delete fsm_control;

    lcm_->unsubscribe(pose_sub);
    lcm_->unsubscribe(stereo_sub);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(filter) = "StateMachineControl*";
  return RUN_ALL_TESTS();
}


#ifndef HUD_OBJECT_DRAWER_HPP
#define HUD_OBJECT_DRAWER_HPP

#include "opencv2/opencv.hpp"
#include <cv.h>
#include <iostream>
#include <string>
#include <bot_core/bot_core.h>
#include "../../controllers/TrajectoryLibrary/TrajectoryLibrary.hpp"
#include "../../sensors/stereo/opencv-stereo-util.hpp"
#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/mav_pose_t.h"

using namespace cv;

#define PI 3.14159265

#define MIN_DISPLAY_THRESHOLD 0.25

class HudObjectDrawer {

    public:
        HudObjectDrawer(const TrajectoryLibrary *trajlib, BotFrames *bot_frames, const OpenCvStereoCalibration *stereo_calibration, bool show_unremapped = false);

        void SetAutonomous(int autonomous) { is_autonomous_ = autonomous == 1; }
        void SetTrajectoryNumber(int traj_number);
        void SetPose(const mav_pose_t *msg);

        void DrawTrajectory(Mat hud_img);

        void DrawObstacles(Mat hud_img, std::vector<Point3f> obstacles);

    private:
        double current_t_;
        int traj_number_;
        bool show_unremapped_;

        Scalar hud_color_ = Scalar(0.45, 0.95, 0.48);
        BotFrames *bot_frames_;
        const TrajectoryLibrary *trajlib_ = nullptr;
        const OpenCvStereoCalibration *stereo_calibration_ = nullptr;

        std::vector<Point2f> DrawBox(Mat hud_img, double xyz[3], double rpy[3], double width, double height, Scalar color);
        void DrawCube(Mat hud_img, double xyz[3], double rpy[3], double width, double height, double length, Scalar color);

        void InitializeState(const mav_pose_t *msg);
        Eigen::VectorXd GetStateMinusInit(const mav_pose_t *msg);

        mav_pose_t *current_pose_msg_ = nullptr;

        int64_t t0_;
        bool state_initialized_ = false;
        bool is_autonomous_ = false;
        Eigen::VectorXd initial_state_;
        Eigen::VectorXd last_state_; // keep so we can do angle unwrapping
        Eigen::Matrix3d Mz_; // rotation matrix that transforms global state into local state by removing yaw

};

#endif

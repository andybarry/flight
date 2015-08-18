#include "HudTrajectoryDrawer.hpp"

HudTrajectoryDrawer::HudTrajectoryDrawer(const TrajectoryLibrary *trajlib, BotFrames *bot_frames, const OpenCvStereoCalibration *stereo_calibration) {
   trajlib_ = trajlib;
   bot_frames_ = bot_frames;
   stereo_calibration_ = stereo_calibration;
   traj_number_ = -1;
}

void HudTrajectoryDrawer::SetTrajectoryNumber(int traj_number) {
    traj_number_ = traj_number;
    state_initialized_ = false;
}

void HudTrajectoryDrawer::SetPose(const mav_pose_t *msg) {
    if (current_pose_msg_ != nullptr) {
        mav_pose_t_destroy(current_pose_msg_);
    }

    current_pose_msg_ = mav_pose_t_copy(msg);
}

void HudTrajectoryDrawer::DrawTrajectory(Mat hud_img) {
    if (traj_number_ < 0 || current_pose_msg_ == nullptr || is_autonomous_ == false) {
        return;
    }

    if (state_initialized_ == false) {
        InitializeState(current_pose_msg_);
    }

    Eigen::VectorXd state_minus_init = GetStateMinusInit(current_pose_msg_);


    // unwrap angles
    state_minus_init(3) = AngleUnwrap(state_minus_init(3), last_state_(3));
    state_minus_init(4) = AngleUnwrap(state_minus_init(4), last_state_(4));
    state_minus_init(5) = AngleUnwrap(state_minus_init(5), last_state_(5));

    last_state_ = state_minus_init;

    const Trajectory *traj = trajlib_->GetTrajectoryByNumber(traj_number_);

    double xyz[3], rpy[3];

    xyz[0] = 5;
    xyz[1] = 0;
    xyz[2] = 0;

    rpy[0] = 0;
    rpy[1] = 0;
    rpy[2] = 0.78;

    //DrawBox(hud_img, xyz, rpy, 0.84, 0.5);

    vector<Point3f> points;
    points.push_back(Point3f(0, 0, 0));

    //Draw3DPointsOnImage(hud_img, &points, stereo_calibration_->M1, stereo_calibration_->D1, stereo_calibration_->R1);

    for (double t = current_t_; t < std::min(traj->GetMaxTime(), 100000.0+current_t_+1.0); t += 0.10) {
        Eigen::VectorXd traj_x = traj->GetState(t);

        Eigen::VectorXd state = traj_x - state_minus_init;

        xyz[0] = state(0);
        xyz[1] = state(1);
        xyz[2] = state(2);

        rpy[0] = state(3);
        rpy[1] = state(4);
        rpy[2] = state(5);

        DrawBox(hud_img, xyz, rpy, 0.84, .5);
    }
}

/**
 * Draws a 2D box transformed by trans, which allows for trajectories
 * to look 3D by shrinking, moving, and rotating them.
 */
void HudTrajectoryDrawer::DrawBox(Mat hud_img, double xyz[3], double rpy[3], double width, double height) {

    BotTrans body_to_stereo;
    bot_frames_get_trans(bot_frames_, "body", "opencvFrame", &body_to_stereo);


    // move the point into the camera frame
    double xyz_camera_frame[3];
    bot_trans_apply_vec(&body_to_stereo, xyz, xyz_camera_frame);

    double rotmat_array[9], quat[4];

    bot_roll_pitch_yaw_to_quat(rpy, quat);
    bot_quat_to_matrix(quat, rotmat_array);

    Eigen::Matrix3d rotmat;
    rotmat << rotmat_array[0], rotmat_array[1], rotmat_array[2],
              rotmat_array[3], rotmat_array[4], rotmat_array[5],
              rotmat_array[6], rotmat_array[7], rotmat_array[8];

    // initialize the corners of the box
    //double box_x[4] = { -width/2, width/2, width/2, -width/2 };
    //double box_y[4] = { -height/2, -height/2, height/2, height/2 };
    //double box_z[4] = { 0, 0, 0, 0 };
    double box_x[4] = { 0, 0, 0, 0 };
    double box_y[4] = { -width/2, width/2, width/2, -width/2 };
    double box_z[4] = { -height/2, -height/2, height/2, height/2 };


    vector<Point3f> box_points;

    for (int i = 0; i < 4; i++) {
        // create the box
        Eigen::Vector3d point_eigen;
        point_eigen << box_x[i], box_y[i], box_z[i];

        // rotate the box point while it is at the origin
        point_eigen = rotmat*point_eigen;

        double point[3] = { point_eigen(0), point_eigen(1), point_eigen(2) };
        double point_camera_frame[3];

        bot_trans_apply_vec(&body_to_stereo, point, point_camera_frame);

        // translate the box to it's 3D location in the camera frame
        point_camera_frame[0] += xyz_camera_frame[0];
        point_camera_frame[1] += xyz_camera_frame[1];
        point_camera_frame[2] += xyz_camera_frame[2];

        if (point_camera_frame[2] < MIN_DISPLAY_THRESHOLD) {
            // this point is behind us, so don't display the box
            return;
        }

        box_points.push_back(Point3f(point_camera_frame[0], point_camera_frame[1], point_camera_frame[2]));
    }

    vector<Point2f> projected_box;
    //projectPoints(points_list, cam_mat_r.inv(), Mat::zeros(3, 1, CV_32F), cam_mat_m, cam_mat_d, img_points_list);
    projectPoints(box_points, stereo_calibration_->R1.inv(), Mat::zeros(3, 1, CV_32F), stereo_calibration_->M1, stereo_calibration_->D1, projected_box);

    line(hud_img, projected_box.at(0), projected_box.at(1), hud_color_);
    line(hud_img, projected_box.at(1), projected_box.at(2), hud_color_);
    line(hud_img, projected_box.at(2), projected_box.at(3), hud_color_);
    line(hud_img, projected_box.at(3), projected_box.at(0), hud_color_);

}


void HudTrajectoryDrawer::InitializeState(const mav_pose_t *msg) {

    initial_state_ = PoseMsgToStateEstimatorVector(msg);
    last_state_ = initial_state_;

    // get the yaw from the initial state

    double rpy[3];

    bot_quat_to_roll_pitch_yaw(msg->orientation, rpy);

    Mz_ = rotz(-rpy[2]);

    t0_ = GetTimestampNow();

    state_initialized_ = true;

}

Eigen::VectorXd HudTrajectoryDrawer::GetStateMinusInit(const mav_pose_t *msg) {

    // subtract out x0, y0, z0
    mav_pose_t *msg2 = mav_pose_t_copy(msg);

    msg2->pos[0] -= initial_state_(0); // x
    msg2->pos[1] -= initial_state_(1); // y
    msg2->pos[2] -= initial_state_(2); // z

    Eigen::VectorXd state = PoseMsgToStateEstimatorVector(msg2, Mz_);

    mav_pose_t_destroy(msg2);

    return state;

}

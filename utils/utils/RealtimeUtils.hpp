#ifndef REALTIME_UTILS_HPP
#define REALTIME_UTILS_HPP

/*
 * Utilities for realtime control
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <boost/algorithm/string/replace.hpp> // for substring replacement

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include "../../LCM/mav_pose_t.h"

#include <Eigen/Core>

#include "gtest/gtest.h"

// for some reason gtest segfaults on the odroids
// when you call EXPECT_EQ
#define EXPECT_EQ_ARM(val1, val2) EXPECT_TRUE(val1 == val2) << "\tval1: " << val1 << std::endl << "\tval2: " << val2 << std::endl;

#define EXPECT_APPROX_MAT(expected_val, got_val, tolerance) EXPECT_TRUE( expected_val.isApprox(got_val, tolerance) ) << std::endl << "Expected:" << std::endl << expected_val << std::endl << "Got:" << std::endl << got_val << std::endl;

/**
 * Converts mav_pose_t message into a 12-state vector in the State Estimator frame.
 * @param msg message to convert
 * @param Mz (optional) yaw rotation
 */
Eigen::VectorXd PoseMsgToStateEstimatorVector(const mav_pose_t *msg, const Eigen::Matrix3d Mz = Eigen::Matrix3d::Identity());

/**
 * Converts mav_pose_t message into the Drake global frame.
 *
 * Supports an optional rotation matrix to rotate about yaw, allowing the user to use a local frame that sets
 * yaw to a specified value.
 */
Eigen::VectorXd StateEstimatorToDrakeVector(const mav_pose_t *msg, const Eigen::Matrix3d Mz = Eigen::Matrix3d::Identity());

/**
 * Computes an angle that removes wrapping and minimizes the jump between the last angle
 * and the current angle, assuming that the difference between them is small.
 *
 * @param angle_rad_in angle input (in radians)
 * @param last_angle_rad last angle (in radians) to attempt to find something close to
 *
 * @retval angle without wrapping
 */
double AngleUnwrap(double angle_rad_in, double last_angle_rad);

Eigen::Matrix3d rpy2rotmat(Eigen::Vector3d rpy);
Eigen::Vector3d angularvel2rpydot(Eigen::Vector3d rpy, Eigen::Vector3d omega);
Eigen::Matrix3d angularvel2rpydotMatrix(Eigen::Vector3d rpy);
Eigen::Matrix3d quat2rotmat(Eigen::Vector4d q);
Eigen::Matrix3d rotz(double rotation_around_z);
Eigen::Vector3d rotmat2rpy(Eigen::Matrix3d rot_mat);

double deg2rad(double input_in_deg);

int64_t GetTimestampNow();

bool NonBlockingLcm(lcm_t *lcm);

void DrawOriginLcmGl(lcm_t *lcm);

std::string ReplaceUserVarInPath(std::string path);

double ConvertTimestampToSeconds(int64_t timestamp);


#endif

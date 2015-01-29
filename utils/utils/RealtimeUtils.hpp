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

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>

#include <Eigen/Core>

#include "gtest/gtest.h"

using namespace std;

Eigen::Matrix3d rpy2rotmat(Eigen::Vector3d rpy);
Eigen::Vector3d angularvel2rpydot(Eigen::Vector3d rpy, Eigen::Vector3d omega);
Eigen::Matrix3d angularvel2rpydotMatrix(Eigen::Vector3d rpy);

int64_t GetTimestampNow();


#endif

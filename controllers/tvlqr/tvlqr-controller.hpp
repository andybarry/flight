#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/mav_pose_t.h"
#include "../../LCM/lcmt_tvlqr_controller_action.h"

#include "../../externals/ConciseArgs.hpp"

#include "../TrajectoryLibrary/TrajectoryLibrary.hpp"

#include "TvlqrControl.hpp"

using namespace std;

void sighandler(int dum);

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

void lcmt_tvlqr_controller_action_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_tvlqr_controller_action *msg, void *user);


// tranform utils

Eigen::Matrix3d rpy2rotmat(Eigen::Vector3d rpy);
Eigen::Vector3d angularvel2rpydot(Eigen::Vector3d rpy, Eigen::Vector3d omega);
Eigen::Matrix3d angularvel2rpydotMatrix(Eigen::Vector3d rpy);


int64_t GetTimestampNow();

Eigen::VectorXd StateEstimatorToDrakeVector(const mav_pose_t *msg);

#endif

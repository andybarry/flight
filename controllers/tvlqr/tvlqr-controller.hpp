#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/mav_pose_t.h"
#include "../../LCM/lcmt_tvlqr_controller_action.h"

#include "../TrajectoryLibrary/TrajectoryLibrary.hpp"

#include "TvlqrControl.hpp"

#include "../../utils/utils/RealtimeUtils.hpp"

using namespace std;

void sighandler(int dum);

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

void lcmt_tvlqr_controller_action_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_tvlqr_controller_action *msg, void *user);


Eigen::VectorXd StateEstimatorToDrakeVector(const mav_pose_t *msg);

#endif

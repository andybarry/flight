#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/mav_pose_t.h"
#include "../../LCM/lcmt_tvlqr_controller_action.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/lcmt_stereo.h"
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"
#include "../../estimators/StereoFilter/StereoFilter.hpp"

#include "../TrajectoryLibrary/TrajectoryLibrary.hpp"

#include "TvlqrControl.hpp"

#include "../../utils/utils/RealtimeUtils.hpp"
#include <bot_param/param_client.h>

#include "lcmtypes/pronto_utime_t.h"
#include "lcmtypes/mav_filter_state_t.h"

#define MAX_SWITCH_MAPPING 100

void sighandler(int dum);

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

void lcmt_tvlqr_controller_action_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_tvlqr_controller_action *msg, void *user);

void pronto_reset_complete_handler(const lcm_recv_buf_t *rbuf, const char* channel, const pronto_utime_t *msg, void *user);

void mav_filter_state_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_filter_state_t *msg, void *user);

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

int ServoToTrajectorySwitchPosition(int servo_value);

void SendStateEstimatorResetRequest();

void SendStateEstimatorDefaultResetRequest();

#endif

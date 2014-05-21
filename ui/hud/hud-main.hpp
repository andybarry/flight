#ifndef UI_PLANE_VISUALIZER
#define UI_PLANE_VISUALIZER

#include "hud.hpp"
#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_optotrak.h"
#include "../../LCM/lcmt_baro_airspeed.h"
#include "../../LCM/lcmt_battery_status.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/mav_gps_data_t.h"
#include "../../LCM/mav_pose_t.h"

#include "lcmtypes/bot_core_image_t.h" // from libbot for images over LCM

#include "../../externals/ConciseArgs.hpp"
#include "../../sensors/stereo/opencv-stereo-util.hpp"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "../../externals/jpeg-utils/jpeg-utils.h"

#include <mutex>

void sighandler(int dum);

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user);

void battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user);

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);

void mav_gps_data_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user);

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user);

void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

bool NonBlockingLcm(lcm_t *lcm);

#endif

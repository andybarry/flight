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
#include <bot_frames/bot_frames.h>

#include "../../externals/jpeg-utils/jpeg-utils.h"

#include <lcmtypes/octomap_raw_t.h>
#include <octomap/OcTree.h>

#include "opencv2/opencv.hpp"
#include <cv.h>

#include <mutex>

#define STEREO_DIST_TO_METERS_DIVISOR 10 // points come in in cm

using namespace std;
using namespace octomap;

void sighandler(int dum);

// stereo handlers
void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);
void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user);
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

// sensor handlers
void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user);
void battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user);
void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);
void mav_gps_data_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user);
void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

// octomap handler
void octomap_raw_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const octomap_raw_t *msg, void *user);

bool GetOctomapPoints(vector<Point3f> *octomap_points, BotTrans *global_to_body);


bool NonBlockingLcm(lcm_t *lcm);

#endif

#ifndef UI_PLANE_VISUALIZER
#define UI_PLANE_VISUALIZER

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

#include "opencv2/opencv.hpp"
#include <cv.h>

#include <mutex>

using namespace std;

void sighandler(int dum);

// stereo handlers
void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user);
void stereo_image_right_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user);

void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);


bool NonBlockingLcm(lcm_t *lcm);

#endif

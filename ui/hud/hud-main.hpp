#ifndef UI_PLANE_VISUALIZER
#define UI_PLANE_VISUALIZER

#include "hud.hpp"
#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_with_xy.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_optotrak.h"
#include "../../LCM/lcmt_battery_status.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/mav_gps_data_t.h"
#include "../../LCM/mav_pose_t.h"

#include "lcmtypes/bot_core_image_t.h" // from libbot for images over LCM
#include "lcmtypes/mav_airspeed_t.h" // from pronto

#include "../../externals/ConciseArgs.hpp"
#include "../../sensors/stereo/opencv-stereo-util.hpp"
#include "../../sensors/stereo/RecordingManager.hpp"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include "../../externals/jpeg-utils/jpeg-utils.h"

#include <lcmtypes/octomap_raw_t.h>
#include <octomap/OcTree.h>
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"

#include "opencv2/opencv.hpp"
#include <cv.h>

#include <mutex>
#include <fstream>

using namespace std;
using namespace octomap;

void sighandler(int dum);

// stereo handlers
void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);
void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user);
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);
void stereo_xy_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_with_xy *msg, void *user);

void stereo_bm_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

// sensor handlers
void mav_airspeed_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_airspeed_t *msg, void *user);
void battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user);
void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);
void mav_gps_data_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user);
void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

// octomap handler
void octomap_raw_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const octomap_raw_t *msg, void *user);

void Get2DPointsFromLcmXY(lcmt_stereo_with_xy *msg, vector<Point> *xy_points);
void Draw2DPointsOnImage(Mat image, vector<Point> *points);

void OnMouse( int event, int x, int y, int flags, void* hud_in);
void ResetBoxDrawing();
void AskForFrame(int video_number, int frame_number);

bool NonBlockingLcm(lcm_t *lcm);

#endif

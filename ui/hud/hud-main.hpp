#ifndef UI_PLANE_VISUALIZER
#define UI_PLANE_VISUALIZER

#include "hud.hpp"
#include "HudObjectDrawer.hpp"
#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_with_xy.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_optotrak.h"
#include "../../LCM/lcmt_battery_status.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/lcmt_tvlqr_controller_action.h"
#include "../../LCM/mav_gps_data_t.h"
#include "../../LCM/mav_pose_t.h"
#include "../../LCM/lcmt_debug.h"

#include "lcmtypes/bot_core_image_t.h" // from libbot for images over LCM
#include "lcmtypes/mav_indexed_measurement_t.h" // from pronto

#include "../../externals/ConciseArgs.hpp"
#include "../../sensors/stereo/opencv-stereo-util.hpp"
#include "../../sensors/stereo/RecordingManager.hpp"
#include "../../utils/ServoConverter/ServoConverter.hpp"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include "../../externals/jpeg-utils/jpeg-utils.h"

#include "opencv2/opencv.hpp"
#include <cv.h>

#include <mutex>
#include <fstream>

#include "../../controllers/TrajectoryLibrary/TrajectoryLibrary.hpp"
#include "../../utils/utils/RealtimeUtils.hpp"

void sighandler(int dum);

// stereo handlers
void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);
void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user);
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);
void mono_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);
void stereo_xy_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_with_xy *msg, void *user);

void stereo_bm_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

// sensor handlers
void airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user);

void airspeed_unchecked_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user);

void battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user);
void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);
void mav_gps_data_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user);
void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);

// trajectory handler
void tvlqr_action_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_tvlqr_controller_action *msg, void *user);

void Get2DPointsFromLcmXY(const lcmt_stereo_with_xy *msg, vector<Point> *xy_points);
void Draw2DPointsOnImage(Mat image, const vector<Point> *points);

void OnMouse( int event, int x, int y, int flags, void* hud_in);
void ResetBoxDrawing();
void AskForFrame(int video_number, int frame_number);


void state_machine_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_debug *msg, void *user);

#endif

#ifndef FPGA_PLAYBACK_HPP
#define FPGA_PLAYBACK_HPP

#include "../../LCM/lcmt_deltawing_u.h"

#include "lcmtypes/bot_core_image_t.h" // from libbot for images over LCM

#include "../../externals/ConciseArgs.hpp"
#include "../../sensors/stereo/opencv-stereo-util.hpp"
#include "../../sensors/stereo/RecordingManager.hpp"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include "../../externals/jpeg-utils/jpeg-utils.h"

#include "opencv2/opencv.hpp"
#include <cv.h>

#include <mutex>
#include <fstream>

// header-only csv parsing library from https://code.google.com/p/fast-cpp-csv-parser/
#include "csv.h"

#include <boost/format.hpp>

void sighandler(int dum);

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);

int GetNearestFpgaFrame(long timestamp);
int GetNearestFpgaFrameFromTable(long timestamp);

string GetFpgaImageFilename(int frame_number);


bool NonBlockingLcm(lcm_t *lcm);

#endif

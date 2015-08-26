#ifndef MONO_PLAYBACK_HPP
#define MONO_PLAYBACK_HPP

#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/lcmt_stereo.h"
#include "../../sensors/stereo/opencv-stereo-util.hpp"
#include "../../LCM/lcmt_cpu_info.h"
#include <boost/format.hpp>


void sighandler(int dum);

void mono_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

void cpu_info_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_cpu_info *msg, void *user);

std::string GetMonoFilename(long timestamp, int video_number);

#endif

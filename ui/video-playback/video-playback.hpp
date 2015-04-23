#ifndef VIDEO_PLAYBACK_HPP
#define VIDEO_PLAYBACK_HPP

#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/lcmt_deltawing_u.h"


void sighandler(int dum);

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);


#endif

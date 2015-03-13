#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/lcmt_midi.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/lcmt_debug.h"
#include "../../utils/utils/RealtimeUtils.hpp"

#include "lcmtypes/mav_indexed_measurement_t.h" // from pronto

#include <bot_param/param_client.h>

void sighandler(int dum);

void lcmt_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user);

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);


#endif

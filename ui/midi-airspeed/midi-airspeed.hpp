#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/lcmt_midi.h"
#include "../../utils/utils/RealtimeUtils.hpp"

#include "lcmtypes/mav_indexed_measurement_t.h" // from pronto

#include <eigen_utils/eigen_utils.hpp> // from pronto

#include <bot_param/param_client.h>

void sighandler(int dum);

void lcmt_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user);

void altimeter_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user);


#endif

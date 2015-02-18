#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/lcmt_midi.h"
#include "../../utils/utils/RealtimeUtils.hpp"

#include "lcmtypes/mav_airspeed_t.h"
#include "lcmtypes/mav_altimeter_t.h"


using namespace std;

void sighandler(int dum);

void lcmt_midi_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_midi *msg, void *user);

void mav_altimeter_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_altimeter_t *msg, void *user);


#endif

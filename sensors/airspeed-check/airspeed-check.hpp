#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/lcmt_debug.h"
#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../utils/RollingStatistics/RollingStatistics.hpp"

#include "lcmtypes/mav_gps_data_t.h" // from pronto
#include "lcmtypes/mav_indexed_measurement_t.h" // from pronto

#include <bot_param/param_client.h>

#include <eigen_utils/eigen_utils.hpp> // from pronto

void sighandler(int dum);

void airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_indexed_measurement_t *msg, void *user);

void gps_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user);

void SendNewAirspeedMessage(double new_speed, const mav_indexed_measurement_t *old_msg);
void SendExistingAirspeedMessage(const mav_indexed_measurement_t *msg);

void SendDebugMessage(char *debug_str);

#endif

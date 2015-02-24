#ifndef ARDUPILOT_MAVLINK_BRIDGE_HPP
#define ARDUPILOT_MAVLINK_BRIDGE_HPP

/*
 * Bridges MAVLINK LCM messages and libbot LCM messages, using the ardupilot
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2015
 *
 */

#include <iostream>


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "../../LCM/mavlink_msg_container_t.h"
#include "../../LCM/lcmt_gps.h"
#include "../../LCM/lcmt_battery_status.h"
#include "../../LCM/lcmt_attitude.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_beep.h"
#include "../../LCM/lcmt_tvlqr_controller_action.h"


#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "lcmtypes/mav_ins_t.h" // from pronto
#include "lcmtypes/mav_gps_data_t.h" // from pronto
#include "lcmtypes/mav_indexed_measurement_t.h" // from pronto

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp> // from pronto



#include "../../externals/ConciseArgs.hpp"

#include "mavconn.h" // from mavconn

//#include "../../mavlink-generated/ardupilotmega/mavlink.h"
#include "../../mavlink-generated2/csailrlg/mavlink.h"
//#include "../../mavlink-generated2/csailrlg/mavlink_msg_scaled_pressure_and_airspeed.h"

#define GRAVITY_MSS 9.80665f // this matches the ArduPilot definition

/* XXX XXX XXX XXX
 *
 * You must add
 *      #include "../mavlink-generated2/ardupilotmega/mavlink.h"
 * to
 *      ../../LCM/mavlink_message_t.h
 * and comment out its definition of
 *  mavlink_message to make this work
 *
 * XXX XXX XXX XXX
 */


void sighandler(int dum);
int64_t getTimestampNow();

void beep_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_beep *msg, void *user);

void deltawing_u_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);

void mavlink_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mavlink_msg_container_t *msg, void *user);

#endif


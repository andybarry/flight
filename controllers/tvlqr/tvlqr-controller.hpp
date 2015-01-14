#ifndef TVLQR_CONTROLLER_HPP
#define TVLQR_CONTROLLER_HPP

#include "../../LCM/mav_pose_t.h"

#include "../../externals/ConciseArgs.hpp"

#include "../TrajectoryLibrary/TrajectoryLibrary.hpp"

using namespace std;

void sighandler(int dum);

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);


#endif

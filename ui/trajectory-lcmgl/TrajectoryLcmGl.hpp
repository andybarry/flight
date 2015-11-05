#ifndef TRAJECTORY_LCMGL_HPP
#define TRAJECTORY_LCMGL_HPP

#include <iostream>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/bot_core.h>
#include "../../controllers/TrajectoryLibrary/TrajectoryLibrary.hpp"
#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/lcmt/tvlqr_controller_action.hpp"

class TrajectoryLcmGl {

    public:
        TrajectoryLcmGl(lcm::LCM *lcm, const TrajectoryLibrary *trajlib);

        void ProcessTrajectoryMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::tvlqr_controller_action *msg);

    private:

        void DrawTrajectoryLcmGl(int traj_number, int64_t timestamp);

        lcm::LCM *lcm_;
        const TrajectoryLibrary *trajlib_;
        BotFrames *bot_frames_;
        std::string last_traj_name_ = "";
        BotTrans last_draw_transform_;
        int64_t last_traj_timestamp_;
        const Trajectory *last_traj_;


};

#endif
